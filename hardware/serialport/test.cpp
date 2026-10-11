#include "hardware/serialport/serialport.hpp"
#include "tools/config/config.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

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
std::filesystem::path write_serial_yaml(const std::string &port, const std::string &extra = "")
{
    const auto    path = std::filesystem::temp_directory_path() / "rm_vision_serialport_hw.yaml";
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
           "  timestamp_offset: 0.0\n"
        << extra;
    return path;
}

} // namespace

int main()
{
    using hardware::serialport::ReceivePacket;
    using hardware::serialport::SendPacket;
    using hardware::serialport::SerialPort;

    const char       *env  = std::getenv("RM_SERIAL_PORT");
    const std::string port = env != nullptr ? env : "/dev/ttyACM0";
    if (!std::filesystem::exists(port))
    {
        std::cout << "real serial skipped: " << port << " not present\n";
        return 0;
    }

    const auto path = write_serial_yaml(port);
    SerialPort serial(tools::config::Config(path.string()));
    check(serial.is_open(), "打开真实串口");

    // 接收测试
    {
        const auto    start = std::chrono::steady_clock::now();
        unsigned      count = 0;
        ReceivePacket last{};
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < 2.0)
        {
            ReceivePacket packet{};
            if (serial.frames().wait_for(packet, std::chrono::milliseconds(5)))
            {
                last = packet;
                ++count;
            }
        }
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        std::printf("rx %u frames in %.2fs = %.1f Hz\n", count, elapsed, static_cast<double>(count) / elapsed);

        check(count > 0, "received frames");
        check(std::fabs(last.quaternion().norm() - 1.0F) < 0.05F, "received quaternion is close to unit");
        check(std::fabs(serial.quaternion_at(tools::time::now()).norm() - 1.0F) < 0.1F, "pose interpolation buffer is available");
    }

    // 发送测试
    {
        serial.send(SendPacket{1.25F, 0.5F, 1});

        unsigned   count = 0;
        const auto start = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < 0.5)
        {
            ReceivePacket packet{};
            if (serial.frames().wait_for(packet, std::chrono::milliseconds(5)))
            {
                ++count;
            }
        }
        std::printf("after tx: %u frames in 0.5s\n", count);

        check(serial.is_open(), "after tx, serial still open");
        check(count > 0, "after tx, received frames");
    }

    std::filesystem::remove(path);

    if (failures != 0)
    {
        std::cerr << "serialport test failed: " << failures << "\n";
        return 1;
    }
    std::cout << "serialport test passed\n";
    return 0;
}
