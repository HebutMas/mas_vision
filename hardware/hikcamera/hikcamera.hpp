#pragma once

#include "tools/config/config.hpp"
#include "tools/latest_frame/latest_frame.hpp"
#include "tools/time/time.hpp"

#include <opencv2/core.hpp>

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

namespace hardware::hikcamera
{

// 采集时刻的时间戳
using TimePoint = tools::time::TimePoint;

// Bayer 去马赛克质量:bilinear 快;edge_aware 边缘自适应,质量高但更慢。
enum class DemosaicQuality : std::uint8_t
{
    bilinear,
    edge_aware,
};

// 相机侧裁切(ROI)
// Bayer 格式要求 width / height 为偶数;width/height<=0 表示用整幅。
struct RoiConfig
{
    bool enable{false};
    int  x{0};
    int  y{0};
    int  width{0};
    int  height{0};
};

// 海康相机配置
struct HikCameraConfig
{
    // 相机序列号;留空则使用枚举到的第一台 USB 相机。
    std::string serial;
    // 曝光时间(微秒)。
    double exposure_us{3000.0};
    // 模拟增益(dB)。
    double gain_db{0.0};
    // 目标帧率(fps);<=0 表示不限制,跑相机最大帧率。
    double framerate{0.0};
    // 相机侧裁切。
    RoiConfig roi;
    // 去马赛克质量。
    DemosaicQuality demosaic{DemosaicQuality::edge_aware};
};

// 把配置字符串转成去马赛克枚举;未知值回落到 edge_aware。
[[nodiscard]] inline DemosaicQuality parse_demosaic(const std::string &name)
{
    return name == "bilinear" ? DemosaicQuality::bilinear : DemosaicQuality::edge_aware;
}

// 从配置读取海康相机参数。
[[nodiscard]] inline HikCameraConfig load_hikcamera_config(const tools::config::Config &config)
{
    HikCameraConfig cfg;
    cfg.serial      = config.value<std::string>("hikcamera.serial", "");
    cfg.exposure_us = config.require<double>("hikcamera.exposure_us");
    cfg.gain_db     = config.require<double>("hikcamera.gain_db");
    cfg.framerate   = config.value<double>("hikcamera.framerate", 0.0);
    cfg.roi.enable  = config.value<bool>("hikcamera.roi.enable", false);
    cfg.roi.x       = config.value<int>("hikcamera.roi.x", 0);
    cfg.roi.y       = config.value<int>("hikcamera.roi.y", 0);
    cfg.roi.width   = config.value<int>("hikcamera.roi.width", 0);
    cfg.roi.height  = config.value<int>("hikcamera.roi.height", 0);
    cfg.demosaic    = parse_demosaic(config.value<std::string>("hikcamera.demosaic", "edge_aware"));
    return cfg;
}

// 海康相机的一帧:图像 + 出流时刻。
struct HikFrame
{
    cv::Mat   image;
    TimePoint timestamp;
};

class HikCamera
{
  public:
    explicit HikCamera(HikCameraConfig config);
    ~HikCamera();

    HikCamera(const HikCamera &)            = delete;
    HikCamera &operator=(const HikCamera &) = delete;

    // 相机是否已打开并开始取流。
    [[nodiscard]] bool is_open() const noexcept;

    // 采集线程持续写入最新帧;消费者用 wait / wait_for / try_pop 取走。
    [[nodiscard]] tools::LatestFrame<HikFrame> &frames() noexcept { return frames_; }

  private:
    void open();
    void close() noexcept;
    void configure();
    void run();

    HikCameraConfig config_;          // 相机配置。
    void           *handle_{nullptr}; // MVS 设备句柄。

    std::thread       thread_;      // 采集线程。
    std::atomic<bool> quit_{false}; // 采集线程退出标志。

    tools::LatestFrame<HikFrame> frames_; // 最新帧槽。
};

} // namespace hardware::hikcamera
