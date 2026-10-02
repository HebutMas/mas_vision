#include "apps/infantry/main_debug.hpp"

#ifdef RM_DEBUG
#include "modules/auto_armor/debug/visualize.hpp"
#include "modules/auto_buff/debug/visualize.hpp"
#include "tools/debug/video/video_encoder.hpp"

#include <opencv2/imgproc.hpp>

#include <memory>
#include <vector>
#endif

#include <chrono>
#include <exception>
#include <string>
#include <utility>

namespace infantry
{
namespace
{
#ifdef RM_DEBUG
// 左上角显示本次推理耗时(ms)。
void draw_latency(cv::Mat &image, double latency_ms)
{
    if (image.empty())
    {
        return;
    }
    const std::string label      = cv::format("detect %.2f ms", latency_ms);
    const double      font_scale = image.rows / 720.0;
    const int         thickness  = 2 + (image.rows / 1080);
    int               baseline   = 0;
    const cv::Size    text_size  = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
    const cv::Point   origin(12, 12 + text_size.height);
    // 先铺一块黑底
    cv::rectangle(image, cv::Rect(origin.x - 6, origin.y - text_size.height - 6, text_size.width + 12, text_size.height + baseline + 12), {0, 0, 0},
                  cv::FILLED);
    cv::putText(image, label, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, {0, 255, 255}, thickness);
}

// 把云台串口数据(模式 + 四元数 + 欧拉角)一并记录到 Rerun。
void log_serial(tools::debug::Sink &sink, const hardware::serialport::ReceivePacket &packet)
{
    sink.data("serial/mode", static_cast<double>(packet.mode));
    sink.data("serial/quaternion", std::vector<double>{packet.qw, packet.qx, packet.qy, packet.qz});
    // ZYX 顺序:分别对应 yaw / pitch / roll。
    const Eigen::Vector3f rpy     = packet.quaternion().toRotationMatrix().canonicalEulerAngles(2, 1, 0);
    constexpr double      RAD2DEG = 57.29577951308232;
    sink.data("serial/euler_deg", std::vector<double>{rpy[0] * RAD2DEG, rpy[1] * RAD2DEG, rpy[2] * RAD2DEG});
}
#endif
} // namespace

Debug::Debug(const std::string &application_id) : sink_(application_id)
{
#ifdef RM_DEBUG
    debug_thread_ = std::thread([this] {
        std::unique_ptr<tools::video::VideoEncoder> encoder;
        DebugFrame                                  f;
        // 阻塞等待最新帧;只有 Debug 析构 close() 时才返回 false 退出。
        while (debug_in_.wait(f))
        {
            if (!sink_.active())
            {
                continue;
            }
            const std::chrono::nanoseconds timestamp = tools::time::since_base(f.timestamp);
            sink_.set_frame(f.frame);
            sink_.set_time("time", timestamp);
            log_serial(sink_, f.serial);
            rm::armor::draw(f.image, f.detection);
            rm::buff::draw(f.image, f.rune_detection);
            draw_latency(f.image, f.latency_ms);
            try
            {
                if (!encoder)
                {
                    tools::video::VideoEncoderConfig encoder_config;
                    encoder_config.width  = f.image.cols;
                    encoder_config.height = f.image.rows;
                    encoder               = std::make_unique<tools::video::VideoEncoder>(encoder_config);
                    tools::debug::log(tools::debug::Level::info,
                                      std::string("debug video encoder: ") + (encoder->hardware() ? "VAAPI (GPU)" : "CPU (libx264)"));
                }
                for (auto &encoded : encoder->encode(f.image, timestamp))
                {
                    sink_.video("camera/image", std::move(encoded.data), encoded.keyframe);
                }
            }
            catch (const std::exception &error)
            {
                // 无 VAAPI 设备等异常:跳过视频输出,不算失败。
                tools::debug::log(tools::debug::Level::warn, std::string("skip camera video output: ") + error.what());
                encoder.reset();
            }
        }
    });
#endif
}

Debug::~Debug()
{
#ifdef RM_DEBUG
    debug_in_.close();
    if (debug_thread_.joinable())
    {
        debug_thread_.join();
    }
#endif
}

bool Debug::active() const { return sink_.active(); }

void Debug::push(const cv::Mat &image, std::int64_t frame, tools::time::TimePoint timestamp, const hardware::serialport::ReceivePacket &serial,
                 rm::armor::Detector::Result detection, double latency_ms)
{
#ifdef RM_DEBUG
    if (sink_.active())
    {
        debug_in_.push(DebugFrame{image, frame, timestamp, serial, std::move(detection), latency_ms, {}});
    }
#else
    (void)image;
    (void)frame;
    (void)timestamp;
    (void)serial;
    (void)detection;
    (void)latency_ms;
#endif
}

void Debug::push(const cv::Mat &image, std::int64_t frame, tools::time::TimePoint timestamp, const hardware::serialport::ReceivePacket &serial,
                 rm::buff::Detector::Result detection, double latency_ms)
{
#ifdef RM_DEBUG
    if (sink_.active())
    {
        debug_in_.push(DebugFrame{image, frame, timestamp, serial, {}, latency_ms, std::move(detection)});
    }
#else
    (void)image;
    (void)frame;
    (void)timestamp;
    (void)serial;
    (void)detection;
    (void)latency_ms;
#endif
}

} // namespace infantry
