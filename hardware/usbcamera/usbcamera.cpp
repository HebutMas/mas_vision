#include "hardware/usbcamera/usbcamera.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <utility>

namespace hardware::usbcamera
{
namespace
{
// V4L2 只保留最新帧,给取图留 1 帧缓冲。
constexpr int BUFFER_SIZE = 1;

// "MJPG" -> FourCC 整数。
int to_fourcc(const std::string & text)
{
  if (text.size() != 4)
  {
    throw std::invalid_argument("UsbCamera fourcc 必须是 4 个字符,当前为 \"" + text + "\"");
  }
  return cv::VideoWriter::fourcc(text[0], text[1], text[2], text[3]);
}
} // namespace

UsbCamera::UsbCamera(UsbCameraConfig config) : config_(std::move(config))
{
  open();
  thread_ = std::thread(&UsbCamera::run, this);
}

UsbCamera::~UsbCamera()
{
  quit_ = true;
  frames_.close();
  if (thread_.joinable())
  {
    thread_.join();
  }
  close();
}

bool UsbCamera::is_open() const noexcept
{
  return capture_.isOpened();
}

void UsbCamera::open()
{
  // 固定 V4L2 后端,避免 OpenCV 回退到 GStreamer 带来的不确定性。
  if (!capture_.open(config_.device_index, cv::CAP_V4L2))
  {
    throw std::runtime_error("UsbCamera 无法打开 /dev/video" +
                             std::to_string(config_.device_index));
  }
  configure();
}

void UsbCamera::configure()
{
  if (config_.width > 0)
  {
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
  }
  if (config_.height > 0)
  {
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
  }
  if (config_.frame_rate > 0.0)
  {
    capture_.set(cv::CAP_PROP_FPS, config_.frame_rate);
  }
  if (!config_.fourcc.empty())
  {
    capture_.set(cv::CAP_PROP_FOURCC, to_fourcc(config_.fourcc));
  }
  capture_.set(cv::CAP_PROP_BUFFERSIZE, BUFFER_SIZE);
}

void UsbCamera::close() noexcept
{
  if (capture_.isOpened())
  {
    capture_.release();
  }
}

void UsbCamera::run()
{
  cv::Mat image;
  while (!quit_)
  {
    // read 返回后立即记录出流时刻。
    if (!capture_.read(image) || image.empty())
    {
      // 掉帧或设备异常:退避后重试,避免空转。
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }
    const TimePoint stamp = tools::time::now();
    frames_.push(UsbFrame{image.clone(), stamp});
  }
}

} // namespace hardware::usbcamera
