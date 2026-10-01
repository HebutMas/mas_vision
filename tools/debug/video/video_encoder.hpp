#pragma once

#include <opencv2/core.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace tools::video
{

// VAAPI H.264 硬编码配置。默认面向 1440x1080@200 的调试视频流。
struct VideoEncoderConfig
{
    int          width       = 1440;
    int          height      = 1080;
    int          fps         = 200;       // 时间基与码率控制的参考帧率
    std::int64_t bitrate     = 8'000'000; // 目标码率 bit/s(CBR);100Mbps 网络下 8M 足够
    int          gop         = 200;       // 关键帧间隔(帧),200 = 1s @200fps
    int          async_depth = 2;         // 编码流水线深度:2 可掩盖 GPU 延迟,CPU 约为深度 1 的 2/3
    std::string  device;                  // VAAPI 渲染节点
};

// 一个编码后的访问单元(Annex B 码流),恰好对应一帧,可直接作为 Rerun VideoStream 的一个 sample。
struct EncodedFrame
{
    std::vector<std::uint8_t> data;
    bool                      keyframe = false;
    std::chrono::nanoseconds  timestamp{}; // 与 encode() 传入的时间一一对应
};

// H.264 编码器:优先核显 VAAPI 硬编;硬编不可用时回退 CPU 软编(libx264)。
// 两条路都不可用时构造抛 std::runtime_error。
class VideoEncoder
{
  public:
    explicit VideoEncoder(const VideoEncoderConfig &config);
    ~VideoEncoder();
    VideoEncoder(const VideoEncoder &)            = delete;
    VideoEncoder &operator=(const VideoEncoder &) = delete;

    // 编码一帧 BGR。返回本次能取出的所有访问单元
    std::vector<EncodedFrame> encode(const cv::Mat &bgr, std::chrono::nanoseconds timestamp);

    // 冲刷编码器,取出缓存的访问单元。退出前调用一次。
    std::vector<EncodedFrame> flush();

    // 当前使用的是硬件(VAAPI)还是 CPU 软编(libx264)路径。
    [[nodiscard]] bool hardware() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace tools::video
