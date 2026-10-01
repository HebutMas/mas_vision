#include "hardware/serialport/serialport.hpp"

#include "tools/algorithm/crc16.hpp"
#include "tools/debug/debug.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <type_traits>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace hardware::serialport
{

// 载荷结构体必须可拷贝,且发送帧不超上限。
static_assert(std::is_trivially_copyable_v<ReceivePacket>, "ReceivePacket must be trivially copyable.");
static_assert(std::is_trivially_copyable_v<SendPacket>, "SendPacket must be trivially copyable.");
static_assert(sizeof(SendPacket) + 3 <= SerialPort::MAX_FRAME_SIZE, "SendPacket overflows MAX_FRAME_SIZE.");

// 读取并校验 serial 段,打开串口,启动接收线程。
SerialPort::SerialPort(const tools::config::Config &config)
{
    config_.debug            = config.require<bool>("serial.debug");
    config_.port             = config.require<std::string>("serial.port");
    config_.baudrate         = config.require<int>("serial.baudrate");
    config_.bytesize         = config.require<int>("serial.bytesize");
    config_.parity           = config.require<std::string>("serial.parity");
    config_.stopbits         = config.require<int>("serial.stopbits");
    config_.flowcontrol      = config.require<std::string>("serial.flowcontrol");
    config_.timestamp_offset = config.require<double>("serial.timestamp_offset");

    if (config_.bytesize < 5 || config_.bytesize > 8)
    {
        throw std::invalid_argument("serial.bytesize must be 5/6/7/8, got " + std::to_string(config_.bytesize));
    }
    if (config_.parity != "none" && config_.parity != "even" && config_.parity != "odd")
    {
        throw std::invalid_argument("serial.parity must be none/even/odd, got " + config_.parity);
    }
    if (config_.stopbits != 1 && config_.stopbits != 2)
    {
        throw std::invalid_argument("serial.stopbits must be 1/2, got " + std::to_string(config_.stopbits));
    }
    if (config_.flowcontrol != "none" && config_.flowcontrol != "rtscts")
    {
        throw std::invalid_argument("serial.flowcontrol must be none/rtscts, got " + config_.flowcontrol);
    }

    switch (config_.baudrate)
    {
    case 9600:
        config_.speed = B9600;
        break;
    case 19200:
        config_.speed = B19200;
        break;
    case 38400:
        config_.speed = B38400;
        break;
    case 57600:
        config_.speed = B57600;
        break;
    case 115200:
        config_.speed = B115200;
        break;
    case 230400:
        config_.speed = B230400;
        break;
    case 460800:
        config_.speed = B460800;
        break;
    case 921600:
        config_.speed = B921600;
        break;
    default:
        throw std::invalid_argument("serial.baudrate not supported: " + std::to_string(config_.baudrate));
    }

    open();
    if (!is_open())
    {
        tools::debug::log(tools::debug::Level::warn, "serial connect failed: " + config_.port + ", retrying", "serial");
    }
    thread_ = std::thread(&SerialPort::run, this);
}

SerialPort::~SerialPort()
{
    frames_.close(); // 先唤醒阻塞的消费者。
    quit_ = true;
    if (thread_.joinable())
    {
        thread_.join(); // 再停线程:保证线程结束前 frames_ 仍存活。
    }
    disconnect();
}

bool SerialPort::open() noexcept
{
    const int fd = ::open(config_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        return false;
    }

    // 按配置设置 termios:8N1 起,再逐项覆盖数据位 / 校验 / 停止位 / 流控。
    termios tty{};
    if (::tcgetattr(fd, &tty) != 0)
    {
        ::close(fd);
        return false;
    }
    ::cfmakeraw(&tty); // 原始模式:不做任何行/回显处理。
    tty.c_cflag |= (CLOCAL | CREAD);

    tty.c_cflag &= ~CSIZE;
    switch (config_.bytesize)
    {
    case 5:
        tty.c_cflag |= CS5;
        break;
    case 6:
        tty.c_cflag |= CS6;
        break;
    case 7:
        tty.c_cflag |= CS7;
        break;
    default:
        tty.c_cflag |= CS8;
        break;
    }

    if (config_.parity == "none")
    {
        tty.c_cflag &= ~PARENB;
    }
    else
    {
        tty.c_cflag |= PARENB;
        if (config_.parity == "odd")
        {
            tty.c_cflag |= PARODD;
        }
        else
        {
            tty.c_cflag &= ~PARODD;
        }
    }

    if (config_.stopbits == 2)
    {
        tty.c_cflag |= CSTOPB;
    }
    else
    {
        tty.c_cflag &= ~CSTOPB;
    }

    if (config_.flowcontrol == "rtscts")
    {
        tty.c_cflag |= CRTSCTS;
    }
    else
    {
        tty.c_cflag &= ~CRTSCTS;
    }

    tty.c_cc[VMIN]  = 0; // 配合 O_NONBLOCK,读到即返回。
    tty.c_cc[VTIME] = 0;
    ::cfsetispeed(&tty, config_.speed);
    ::cfsetospeed(&tty, config_.speed);
    if (::tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        ::close(fd);
        return false;
    }
    ::tcflush(fd, TCIOFLUSH);

    {
        std::scoped_lock const lock(fd_mutex_);
        fd_.store(fd);
    }
    rx_len_ = 0;     // 丢弃上一条链路的半帧。
    frames_.clear(); // 新连接:丢弃旧链路的收帧与姿态历史。
    imu_.clear();
    try
    {
        tools::debug::log(tools::debug::Level::info, "serial connected: " + config_.port + " @" + std::to_string(config_.baudrate), "serial");
    }
    catch (...)
    {
        static_cast<void>(0);
    }
    return true;
}

bool SerialPort::is_open() const noexcept { return fd_.load() >= 0; }

Eigen::Quaternionf SerialPort::quaternion_at(tools::time::TimePoint time) const { return imu_.at(time); }

void SerialPort::send(const SendPacket &packet)
{
    std::array<std::uint8_t, FRAME_TX_SIZE> frame{};
    frame[0] = TX_HEADER;
    std::memcpy(frame.data() + 1, &packet, sizeof(SendPacket));
    const std::uint16_t crc  = tools::crc16::checksum(frame.data(), 1 + sizeof(SendPacket));
    frame[FRAME_TX_SIZE - 2] = static_cast<std::uint8_t>(crc & 0xFF);
    frame[FRAME_TX_SIZE - 1] = static_cast<std::uint8_t>(crc >> 8);

    std::scoped_lock const lock(fd_mutex_);
    const int              fd = fd_.load();
    if (fd < 0)
    {
        return;
    }
    if (::write(fd, frame.data(), frame.size()) < 0 && errno != EAGAIN && errno != EINTR && errno != EWOULDBLOCK)
    {
        disconnect_locked(); // 链路已坏:置为断开,交给接收线程重连。
    }
}

// 断开当前连接(需持有 fd_mutex_)。
void SerialPort::disconnect_locked() noexcept
{
    const int fd = fd_.exchange(-1);
    if (fd >= 0)
    {
        ::close(fd);
        if (!quit_)
        {
            try
            {
                tools::debug::log(tools::debug::Level::warn, "serial disconnected: " + config_.port + ", reconnecting", "serial");
            }
            catch (...)
            {
                // disconnect_locked() 是 noexcept:日志失败绝不能影响串口本身,静默忽略。
                static_cast<void>(0);
            }
        }
    }
}

void SerialPort::disconnect() noexcept
{
    std::scoped_lock const lock(fd_mutex_);
    disconnect_locked();
}

// 等 5s后重新打开串口
void SerialPort::serialreconnect()
{
    constexpr int STEP_MS = 50;
    for (int elapsed = 0; elapsed < RECONNECT_INTERVAL_MS && !quit_; elapsed += STEP_MS)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(STEP_MS));
    }
    if (!quit_)
    {
        open();
    }
}

// 主循环:poll 到数据就 serialread,链路异常就断开重连。
void SerialPort::run()
{
    while (!quit_)
    {
        if (!is_open())
        {
            serialreconnect();
            continue;
        }

        pollfd    descriptor{fd_.load(), POLLIN, 0};
        const int ready = ::poll(&descriptor, 1, POLL_TIMEOUT_MS);
        if (quit_)
        {
            break;
        }
        if (ready < 0)
        {
            if (errno != EINTR) // EINTR 是信号打断,不是链路故障。
            {
                disconnect();
            }
            continue;
        }
        if (ready == 0)
        {
            continue; // 超时:无数据。
        }
        // 拔出 / 链路复位时 poll 报 HUP / ERR;端口失效报 NVAL。
        if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
        {
            disconnect();
            continue;
        }
        if (!serialread())
        {
            disconnect();
        }
    }
}

bool SerialPort::serialread()
{
    std::array<std::uint8_t, 256> buffer{};
    const ssize_t                 n = ::read(fd_.load(), buffer.data(), buffer.size());
    if (n < 0)
    {
        return errno == EAGAIN || errno == EINTR || errno == EWOULDBLOCK;
    }

    if (config_.debug && n > 0)
    {
        // 打印本次收到的原始字节,便于定位帧头 / CRC 问题(同时进 Rerun)。
        std::string dump = "serial rx:";
        for (ssize_t i = 0; i < n; ++i)
        {
            char byte[8];
            std::snprintf(byte, sizeof(byte), " %02X", buffer[i]);
            dump += byte;
        }
        tools::debug::log(tools::debug::Level::debug, dump);
    }

    for (ssize_t i = 0; i < n; ++i)
    {
        if (rx_len_ == rx_buf_.size())
        {
            // 缓冲填满仍无完整帧:丢弃最旧 1 字节,保证有界。
            std::memmove(rx_buf_.data(), rx_buf_.data() + 1, rx_buf_.size() - 1);
            --rx_len_;
        }
        rx_buf_[rx_len_++] = buffer[i];

        // 扫描缓冲:对齐帧头,校验 CRC;失败则前移 1 字节重扫。
        while (true)
        {
            // 跳过帧头之前的字节
            std::size_t pos = 0;
            while (pos < rx_len_ && rx_buf_[pos] != RX_HEADER)
            {
                ++pos;
            }
            if (pos == rx_len_)
            {
                rx_len_ = 0;
                break;
            }
            if (pos > 0)
            {
                std::memmove(rx_buf_.data(), rx_buf_.data() + pos, rx_len_ - pos);
                rx_len_ -= pos;
            }
            if (rx_len_ < FRAME_RX_SIZE)
            {
                break; // 等待更多字节。
            }

            const auto expected = static_cast<std::uint16_t>(rx_buf_[FRAME_RX_SIZE - 2] | (rx_buf_[FRAME_RX_SIZE - 1] << 8));
            if (tools::crc16::checksum(rx_buf_.data(), FRAME_RX_SIZE - 2) != expected)
            {
                // 假帧头:前移 1 字节重扫。
                std::memmove(rx_buf_.data(), rx_buf_.data() + 1, rx_len_ - 1);
                --rx_len_;
                continue;
            }

            ReceivePacket packet;
            std::memcpy(&packet, rx_buf_.data() + 1, sizeof(ReceivePacket));
            frames_.push(packet);
            // 到达时刻 + timestamp_offset,作为该帧姿态的时间戳存入历史。
            const auto offset = std::chrono::duration<double>(config_.timestamp_offset);
            imu_.push(tools::time::now() + std::chrono::duration_cast<tools::time::TimePoint::duration>(offset), packet.quaternion());
            std::memmove(rx_buf_.data(), rx_buf_.data() + FRAME_RX_SIZE, rx_len_ - FRAME_RX_SIZE);
            rx_len_ -= FRAME_RX_SIZE;
        }
    }
    return true;
}

} // namespace hardware::serialport
