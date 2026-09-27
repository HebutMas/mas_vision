#pragma once

#include "tools/latest_frame/latest_frame.hpp"
#include "tools/time/time.hpp"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <atomic>
#include <string>
#include <thread>

namespace hardware::usbcamera
{

// 采集时刻的时间戳。
using TimePoint = tools::time::TimePoint;

struct UsbCameraConfig
{
  // V4L2 设备序号(/dev/videoN 的 N);。
  int device_index{0};
  // 采集分辨率;0 表示沿用相机默认值。
  int width{1280};
  int height{720};
  // 目标采集帧率(fps);0 表示沿用相机默认值。
  double frame_rate{30.0};
  // 像素格式(FourCC),如 "MJPG"或 "YUYV"。
  std::string fourcc{"MJPG"};
};

// USB 相机的一帧:图像 + 出流时刻。
struct UsbFrame
{
  cv::Mat image;
  TimePoint timestamp;
};

class UsbCamera
{
public:
  explicit UsbCamera(UsbCameraConfig config);
  ~UsbCamera();

  UsbCamera(const UsbCamera &) = delete;
  UsbCamera & operator=(const UsbCamera &) = delete;

  // 相机是否已打开并开始取流。
  [[nodiscard]] bool is_open() const noexcept;

  // 采集线程持续写入最新帧;消费者用 wait / wait_for / try_pop 取走。
  [[nodiscard]] tools::LatestFrame<UsbFrame> & frames() noexcept
  {
    return frames_;
  }

private:
  void open();
  void close() noexcept;
  void configure();
  void run();

  UsbCameraConfig config_;   // 相机配置。
  cv::VideoCapture capture_; // V4L2 取流句柄。

  std::thread thread_;            // 采集线程。
  std::atomic<bool> quit_{false}; // 采集线程退出标志。

  tools::LatestFrame<UsbFrame> frames_; // 采集线程 -> 消费者的最新帧槽。
};

} // namespace hardware::usbcamera
