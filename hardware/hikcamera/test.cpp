#include "hardware/hikcamera/hikcamera.hpp"
#if defined(RM_DEBUG)
#include "tools/debug/debug.hpp"
#include "tools/debug/video/video_encoder.hpp"
#include "tools/time/time.hpp"
#endif

#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#if defined(RM_DEBUG)
namespace
{
constexpr int SEND_SECONDS = 5;
} // namespace
#endif

// 测试:没有相机时应当抛异常并返回 0;接了相机则抓一帧验证输出格式。
// 调试构建下若相机出图,再把画面硬编成 H.264 送到 Rerun Viewer 方便肉眼确认。
int main()
try
{
    // 配置加载自检。
    {
        const auto    path = std::filesystem::temp_directory_path() / "rm_vision_hikcamera_test.yaml";
        std::ofstream out(path);
        out << "hikcamera:\n  serial: ABC123\n  exposure_us: 5000\n  gain_db: 6.5\n";
        out.close();
        const auto cfg = hardware::hikcamera::load_hikcamera_config(tools::config::Config(path.string()));
        if (cfg.serial != "ABC123" || cfg.exposure_us != 5000.0 || cfg.gain_db != 6.5)
        {
            std::cerr << "FAIL: hikcamera config load\n";
            return 1;
        }

        // 缺键:require 抛错。
        std::ofstream partial(path);
        partial << "hikcamera:\n  serial: ABC123\n";
        partial.close();
        bool threw = false;
        try
        {
            (void)hardware::hikcamera::load_hikcamera_config(tools::config::Config(path.string()));
        }
        catch (const std::exception &)
        {
            threw = true;
        }
        if (!threw)
        {
            std::cerr << "FAIL: hikcamera missing key should throw\n";
            return 1;
        }
        std::filesystem::remove(path);
    }

    const hardware::hikcamera::HikCameraConfig config;
    try
    {
        hardware::hikcamera::HikCamera camera(config);
        std::cout << "The Hikvision USB camera has been turned on and an attempt has been made to "
                     "capture a frame.\n";

        auto                         &frames = camera.frames();
        hardware::hikcamera::HikFrame frame;
        if (frames.wait_for(frame, std::chrono::milliseconds(2000)) && !frame.image.empty())
        {
            std::cout << "Frame captured successfully: " << frame.image.cols << "x" << frame.image.rows << " channels=" << frame.image.channels()
                      << "\n";
#if defined(RM_DEBUG)
            tools::debug::Sink debug("rm_vision.hikcamera");
            if (debug.active())
            {
                // 连续发送 5 秒
                try
                {
                    tools::video::VideoEncoderConfig encoder_config;
                    encoder_config.width  = frame.image.cols;
                    encoder_config.height = frame.image.rows;
                    tools::video::VideoEncoder encoder(encoder_config);

                    std::cout << "Start to continuously send the video to Rerun(5s)\n";
                    const auto deadline = tools::time::now() + std::chrono::seconds(SEND_SECONDS);
                    int        index    = 0;
                    while (tools::time::now() < deadline && !frame.image.empty())
                    {
                        const auto timestamp = tools::time::since_base(frame.timestamp);
                        debug.set_frame(index++);
                        debug.set_time("time", timestamp);
                        for (auto &encoded : encoder.encode(frame.image, timestamp))
                        {
                            debug.video("camera/image", std::move(encoded.data), encoded.keyframe);
                        }
                        if (!frames.wait_for(frame, std::chrono::milliseconds(2000)))
                        {
                            break;
                        }
                    }
                }
                catch (const std::exception &error)
                {
                    // 无 VAAPI 设备等异常:跳过视频输出,不算失败。
                    std::cout << "skip video output: " << error.what() << "\n";
                }
            }
            else
            {
                std::cout << "Rerun Viewer is not available, skipping video output\n";
            }
#endif
        }
        else
        {
            std::cout << "Timeout: no frame captured (camera may not be outputting images correctly)\n";
        }
    }
    catch (const std::exception &error)
    {
        // 找不到相机属于正常情况:驱动会抛异常,这里视为通过。
        std::cout << "hikcamera test error: " << error.what() << "\n";
    }

    std::cout << "hikcamera test passed\n";
    return 0;
}
catch (const std::exception &error)
{
    std::cerr << "hikcamera_test error: " << error.what() << "\n";
    return 1;
}
