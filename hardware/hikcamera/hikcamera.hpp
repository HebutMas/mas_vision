#pragma once

#include "tools/latest_frame/latest_frame.hpp"
#include "tools/time/time.hpp"

#include <opencv2/core.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

namespace hardware::hikcamera
{

// 采集时刻的时间戳。
using TimePoint = tools::time::TimePoint;

struct HikCameraConfig
{
  // 相机序列号;留空则使用枚举到的第一台 USB 相机。
  std::string serial;
  // 曝光时间(微秒)。
  double exposure_us{3000.0};
  // 模拟增益(dB)。
  double gain_db{0.0};
  // 目标采集帧率(fps)。
  double frame_rate{150.0};
  // 自动白平衡;关闭时沿用相机当前白平衡。
  bool auto_white_balance{true};
};

// 海康相机的一帧:图像 + 出流时刻。
struct HikFrame
{
  cv::Mat image;
  TimePoint timestamp;
};

class HikCamera
{
public:
  explicit HikCamera(HikCameraConfig config);
  ~HikCamera();

  HikCamera(const HikCamera &) = delete;
  HikCamera & operator=(const HikCamera &) = delete;

  // 相机是否已打开并开始取流。
  [[nodiscard]] bool is_open() const noexcept;

  // 采集线程持续写入最新帧;消费者用 wait / wait_for / try_pop 取走。
  [[nodiscard]] tools::LatestFrame<HikFrame> & frames() noexcept
  {
    return frames_;
  }

private:
  void open();
  void close() noexcept;
  void configure();
  void run();

  HikCameraConfig config_; // 相机配置。
  void * handle_{nullptr}; // MVS 设备句柄。

  std::thread thread_;            // 采集线程。
  std::atomic<bool> quit_{false}; // 采集线程退出标志。

  tools::LatestFrame<HikFrame> frames_; // 采集线程 -> 消费者的最新帧槽。
};

} // namespace hardware::hikcamera
