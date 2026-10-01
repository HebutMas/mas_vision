#include "apps/infantry/main_debug.hpp"

#ifdef RM_DEBUG
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
// 识别颜色 -> BGR 显示色(红/蓝/灰/紫)。
cv::Scalar armor_color_bgr(rm::armor::Color color)
{
    switch (color)
    {
    case rm::armor::Color::red:
        return {0, 0, 255};
    case rm::armor::Color::blue:
        return {255, 0, 0};
    case rm::armor::Color::gray:
        return {128, 128, 128};
    case rm::armor::Color::purple:
        return {255, 0, 255};
    }
    return {255, 255, 255};
}

// 把识别结果画到图上
void draw_detection(cv::Mat &image, const rm::armor::Detector::Result &result)
{
    if (image.empty())
    {
        return;
    }
    for (const auto &armor : result.armors)
    {
        std::vector<cv::Point> polygon;
        polygon.reserve(armor.corners.size());
        for (const auto &corner : armor.corners)
        {
            polygon.emplace_back(cvRound(corner.x), cvRound(corner.y));
        }
        const cv::Scalar color = armor_color_bgr(armor.color);
        cv::polylines(image, polygon, true, color, 2);
        const std::string label = "k" + std::to_string(static_cast<int>(armor.kind)) + " c" + std::to_string(static_cast<int>(armor.color)) + " " +
                                  cv::format("%.2f", armor.confidence);
        // 字体随分辨率放大:720p→1.0/2px,1080p→1.5/3px;原来 0.5/1px 编码后发糊。
        const double font_scale = image.rows / 720.0;
        const int    thickness  = 2 + (image.rows / 1080);
        cv::putText(image, label, polygon.front(), cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
    }
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
            draw_detection(f.image, f.detection);
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
                 rm::armor::Detector::Result detection)
{
#ifdef RM_DEBUG
    if (sink_.active())
    {
        debug_in_.push(DebugFrame{image, frame, timestamp, serial, std::move(detection)});
    }
#else
    (void)image;
    (void)frame;
    (void)timestamp;
    (void)serial;
    (void)detection;
#endif
}

} // namespace infantry
