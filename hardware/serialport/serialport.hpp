#pragma once

#include "hardware/serialport/serialtypes.hpp"
#include "tools/algorithm/quaternion_buffer.hpp"
#include "tools/config/config.hpp"
#include "tools/latest_frame/latest_frame.hpp"
#include "tools/time/time.hpp"

#include <Eigen/Geometry>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <termios.h>

namespace hardware::serialport
{

// 串口连接参数
struct SerialConfig
{
    bool        debug{false};          // 是否打印连接 / 断开日志。
    std::string port{"/dev/gimbal"};   // 设备路径。
    int         baudrate{115200};      // 波特率。
    int         bytesize{8};           // 数据位:5 / 6 / 7 / 8。
    std::string parity{"none"};        // 校验:none / even / odd。
    int         stopbits{1};           // 停止位:1 / 2。
    std::string flowcontrol{"none"};   // 流控:none / rtscts。
    double      timestamp_offset{0.0}; // IMU 时间戳偏移(秒):到达时刻 + offset。
    speed_t     speed{B115200};        // 由 baudrate 解析的 termios 波特率。
};

class SerialPort
{
  public:
    static constexpr std::uint8_t RX_HEADER             = 0x5A;
    static constexpr std::uint8_t TX_HEADER             = 0xA5;
    static constexpr std::size_t  FRAME_RX_SIZE         = 1 + sizeof(ReceivePacket) + 2;
    static constexpr std::size_t  FRAME_TX_SIZE         = 1 + sizeof(SendPacket) + 2;
    static constexpr std::size_t  IMU_BUFFER_CAPACITY   = 256;
    static constexpr int          RECONNECT_INTERVAL_MS = 5000; // 重连间隔。
    static constexpr int          POLL_TIMEOUT_MS       = 200;  // 接收轮询超时。
    static constexpr std::size_t  MAX_FRAME_SIZE        = 128;  // 载荷 + header + CRC 的上限。

    // 读取配置、打开串口并启动接收线程
    explicit SerialPort(const tools::config::Config &config);
    ~SerialPort();

    // 禁止拷贝和赋值，单一实例
    SerialPort(const SerialPort &)            = delete;
    SerialPort &operator=(const SerialPort &) = delete;

    // 串口是否已打开
    [[nodiscard]] bool is_open() const noexcept;

    // 取 time 时刻的云台姿态
    [[nodiscard]] Eigen::Quaternionf quaternion_at(tools::time::TimePoint time) const;

    // 接收线程持续写入最新一帧;消费者用 wait / wait_for / try_pop 取走。
    [[nodiscard]] tools::LatestFrame<ReceivePacket> &frames() noexcept { return frames_; }

    // 发送
    void send(const SendPacket &packet);

  private:
    // 连接
    bool open() noexcept;              // 打开并配置串口
    void disconnect() noexcept;        // 断开当前连接
    void disconnect_locked() noexcept; // 断开当前连接(已锁定 fd_mutex_)
    void serialreconnect();            // 等 5s后重连。

    // 接收线程
    void run();        // 主循环:poll -> serialread,异常则重连
    bool serialread(); // 读一批字节入缓冲并解析成帧

    SerialConfig                                      config_;
    std::array<std::uint8_t, 2 * FRAME_RX_SIZE>       rx_buf_{};    // 接收解析缓冲。
    std::size_t                                       rx_len_{0};   // 缓冲内有效字节数。
    std::atomic<int>                                  fd_{-1};      // 串口 fd;<0 表示未连接。
    std::thread                                       thread_;      // 接收线程。
    std::atomic<bool>                                 quit_{false}; // 接收线程退出标志。
    std::mutex                                        fd_mutex_;    // 保护 fd 的换入 / 关闭,避免 send 与重连竞争。
    tools::TimedQuaternionBuffer<IMU_BUFFER_CAPACITY> imu_;         // IMU 姿态历史。
    tools::LatestFrame<ReceivePacket>                 frames_;      // 接收最新帧。
};

} // namespace hardware::serialport
