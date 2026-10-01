#pragma once

#include "tools/config/config.hpp"
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

// USB 相机配置,与 YAML `usbcamera` 段一一对应。
struct UsbCameraConfig
{
    std::string device_path{"/dev/video0"}; // 设备路径。
    int         width{1280};                // 图像宽度。
    int         height{720};                // 图像高度。
    int         fps{30};                    // 帧率。
    std::string fourcc{"MJPG"};             // 像素格式,如 MJPG / YUYV。
    int         auto_exposure{1};           // 曝光模式:0=自动,1=手动。
    double      exposure{50.0};             // 曝光值(仅手动模式)。
    double      gain{20.0};                 // 增益(仅手动模式)。
    int         auto_wb{0};                 // 自动白平衡:0=禁用,1=启用。
};

// 从配置读取 usbcamera 段(严格:缺键 / 类型不符直接抛错,启动即失败)。
[[nodiscard]] inline UsbCameraConfig load_usbcamera_config(const tools::config::Config &config)
{
    UsbCameraConfig cfg;
    cfg.device_path   = config.require<std::string>("usbcamera.device_path");
    cfg.width         = config.require<int>("usbcamera.width");
    cfg.height        = config.require<int>("usbcamera.height");
    cfg.fps           = config.require<int>("usbcamera.fps");
    cfg.fourcc        = config.require<std::string>("usbcamera.fourcc");
    cfg.auto_exposure = config.require<int>("usbcamera.auto_exposure");
    cfg.exposure      = config.require<double>("usbcamera.exposure");
    cfg.gain          = config.require<double>("usbcamera.gain");
    cfg.auto_wb       = config.require<int>("usbcamera.auto_wb");
    return cfg;
}

// USB 相机的一帧:图像 + 出流时刻。
struct UsbFrame
{
    cv::Mat   image;
    TimePoint timestamp;
};

class UsbCamera
{
  public:
    explicit UsbCamera(UsbCameraConfig config);
    ~UsbCamera();

    UsbCamera(const UsbCamera &)            = delete;
    UsbCamera &operator=(const UsbCamera &) = delete;

    // 相机是否已打开并开始取流。
    [[nodiscard]] bool is_open() const noexcept;

    // 采集线程持续写入最新帧;消费者用 wait / wait_for / try_pop 取走。
    [[nodiscard]] tools::LatestFrame<UsbFrame> &frames() noexcept { return frames_; }

  private:
    void open();
    void close() noexcept;
    void configure();
    void run();

    UsbCameraConfig  config_;  // 相机配置。
    cv::VideoCapture capture_; // V4L2 取流句柄。

    std::thread       thread_;      // 采集线程。
    std::atomic<bool> quit_{false}; // 采集线程退出标志。

    bool stream_ok_{true}; // 上一次 read 是否成功

    tools::LatestFrame<UsbFrame> frames_; // 采集线程 -> 消费者的最新帧槽。
};

} // namespace hardware::usbcamera
