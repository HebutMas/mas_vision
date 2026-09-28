#include "hardware/serialport/serialport.hpp"
#include "tools/algorithm/crc16.hpp"
#include "tools/config/config.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <poll.h>
#include <pty.h>
#include <string>
#include <thread>
#include <unistd.h>

namespace
{
int failures = 0;

// 不依赖 NDEBUG:Release 下 assert 会被编掉,所以自己记失败。
void check(bool ok, const char *what)
{
    if (!ok)
    {
        std::cerr << "FAIL: " << what << "\n";
        ++failures;
    }
}

// 写一份最小可用的 serial 配置(指定端口),返回文件路径。
std::filesystem::path write_serial_yaml(const std::string &port)
{
    const auto    path = std::filesystem::temp_directory_path() / "rm_vision_serialport_port.yaml";
    std::ofstream out(path);
    out << "serial:\n"
           "  debug: false\n"
           "  port: "
        << port
        << "\n"
           "  baudrate: 115200\n"
           "  bytesize: 8\n"
           "  parity: none\n"
           "  stopbits: 1\n"
           "  flowcontrol: none\n"
           "  timestamp_offset: 0.0\n";
    return path;
}
} // namespace

// 用伪终端做回环:验证收发两向的组帧 / 解析 / 重同步。
int main()
{
    using SerialPort    = hardware::serialport::SerialPort;
    using ReceivePacket = hardware::serialport::ReceivePacket;
    using SendPacket    = hardware::serialport::SendPacket;

    int  master    = -1;
    int  slave     = -1;
    char name[128] = {};
    check(::openpty(&master, &slave, name, nullptr, nullptr) == 0, "openpty");
    ::close(slave); // SerialPort 自己打开。

    const auto port_path = write_serial_yaml(name);
    SerialPort serialport(tools::config::Config(port_path.string()));

    // 上行:帧前塞噪声(含假帧头),SerialPort 应校验并重同步解出。
    std::uint8_t frame[1 + sizeof(ReceivePacket) + 2] = {};
    frame[0]                                          = SerialPort::RX_HEADER;
    // mode=7,四元数为绕 X 轴 90°。
    const ReceivePacket expected{7, 0.70710678F, 0.70710678F, 0.0F, 0.0F};
    std::memcpy(frame + 1, &expected, sizeof(ReceivePacket));
    const std::uint16_t crc          = tools::crc16::checksum(frame, 1 + sizeof(ReceivePacket));
    frame[1 + sizeof(ReceivePacket)] = static_cast<std::uint8_t>(crc & 0xFF);
    frame[2 + sizeof(ReceivePacket)] = static_cast<std::uint8_t>(crc >> 8);

    const std::uint8_t noise[] = {0x00, 0x5A, 0x11}; // 最后一个含假帧头。
    check(::write(master, noise, sizeof(noise)) == static_cast<ssize_t>(sizeof(noise)), "write noise");
    check(::write(master, frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame)), "write frame");

    ReceivePacket received{};
    check(serialport.frames().wait_for(received, std::chrono::milliseconds(1000)), "rx timeout");
    check(received.mode == expected.mode, "rx payload");

    // 每帧 Rx 的四元数应进入插值缓冲,可按任意(近期)时刻取回。
    const auto attitude = serialport.quaternion_at(tools::time::now());
    check(std::fabs(attitude.w() - expected.qw) < 1e-4F && std::fabs(attitude.x() - expected.qx) < 1e-4F, "quaternion extracted");

    // 下行:send 后从 master 应读到同格式的帧。
    const SendPacket sent{1.25F, 0.5F, 1};
    serialport.send(sent);

    std::uint8_t buffer[64] = {};
    pollfd       descriptor{master, POLLIN, 0};
    check(::poll(&descriptor, 1, 1000) == 1, "tx poll");
    const ssize_t n = ::read(master, buffer, sizeof(buffer));
    check(n == static_cast<ssize_t>(SerialPort::FRAME_TX_SIZE), "tx frame size");
    check(buffer[0] == SerialPort::TX_HEADER, "tx header");
    SendPacket got{};
    std::memcpy(&got, buffer + 1, sizeof(SendPacket));
    check(got.target_yaw == sent.target_yaw && got.target_pitch == sent.target_pitch && got.fire_advice == sent.fire_advice, "tx payload");
    const std::uint16_t got_crc = tools::crc16::checksum(buffer, 1 + sizeof(SendPacket));
    check((buffer[1 + sizeof(SendPacket)] | (buffer[2 + sizeof(SendPacket)] << 8)) == got_crc, "tx crc");

    // 掉线检测:master 端消失后,SerialPort 应把 is_open 置为 false 并转入重连。
    ::close(master);
    bool closed = false;
    for (int i = 0; i < 40 && !closed; ++i) // 最多等 2s。
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        closed = !serialport.is_open();
    }
    check(closed, "disconnect detected");
    check(!serialport.is_open(), "stays closed while retrying");

    // 设备不存在时构造不抛异常,保持在未连接状态(后台持续重连)。
    {
        const auto missing_path = write_serial_yaml("/dev/rm-nonexistent");
        SerialPort missing(tools::config::Config(missing_path.string()));
        check(!missing.is_open(), "missing device stays closed");
    }

    // YAML 配置:serial 段严格读取,缺键 / 非法值即抛错。
    {
        const auto path = std::filesystem::temp_directory_path() / "rm_vision_serialport_test.yaml";

        // 返回 true 表示构造成功(未抛异常)。
        const auto constructs = [&path](const std::string &yaml) {
            std::ofstream out(path);
            out << yaml;
            out.close();
            try
            {
                hardware::serialport::SerialPort port(tools::config::Config(path.string()));
                return true;
            }
            catch (const std::exception &)
            {
                return false;
            }
        };

        const std::string valid = "serial:\n"
                                  "  debug: false\n"
                                  "  port: /dev/rm-nonexistent\n"
                                  "  baudrate: 921600\n"
                                  "  bytesize: 7\n"
                                  "  parity: even\n"
                                  "  stopbits: 2\n"
                                  "  flowcontrol: rtscts\n"
                                  "  timestamp_offset: 0.5\n";
        check(constructs(valid), "valid serial config constructs");

        check(!constructs("serial:\n  port: /dev/ttyUSB3\n"), "missing serial key throws");
        check(!constructs(valid.substr(0, valid.find("bytesize")) + "bytesize: 9\n"), "invalid bytesize throws");
        check(!constructs(valid.substr(0, valid.find("parity")) + "parity: foo\n"), "invalid parity throws");
        check(!constructs(valid.substr(0, valid.find("baudrate")) + "baudrate: 12345\n"), "unsupported baudrate throws");

        std::filesystem::remove(path);
    }

    if (failures != 0)
    {
        std::cerr << "serialport test failed: " << failures << "\n";
        return 1;
    }
    std::cout << "serialport test passed\n";
    return 0;
}
