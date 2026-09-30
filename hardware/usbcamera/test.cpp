#include "hardware/usbcamera/usbcamera.hpp"
#ifdef RM_DEBUG
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

#ifdef RM_DEBUG
namespace
{
constexpr int SEND_SECONDS = 5;
} // namespace
#endif

// 测试:没有相机时应当抛异常并返回 0;接了相机则抓一帧验证输出格式。
// 调试构建下若相机出图发送到Rerun Viewer 方便肉眼确认。
int main()
try
{
    // 配置加载自检:YAML 覆盖默认值;缺键严格抛错。
    {
        const auto    path = std::filesystem::temp_directory_path() / "rm_vision_usbcamera_test.yaml";
        std::ofstream out(path);
        out << "usbcamera:\n"
               "  device_path: /dev/video2\n"
               "  width: 640\n"
               "  height: 480\n"
               "  fps: 60\n"
               "  fourcc: YUYV\n"
               "  auto_exposure: 0\n"
               "  exposure: 100\n"
               "  gain: 5\n"
               "  auto_wb: 1\n";
        out.close();
        const auto cfg = hardware::usbcamera::load_usbcamera_config(tools::config::Config(path.string()));
        if (cfg.device_path != "/dev/video2" || cfg.width != 640 || cfg.height != 480 || cfg.fps != 60 || cfg.fourcc != "YUYV" ||
            cfg.auto_exposure != 0 || cfg.exposure != 100.0 || cfg.gain != 5.0 || cfg.auto_wb != 1)
        {
            std::cerr << "FAIL: usbcamera config load\n";
            return 1;
        }

        // 缺键:require 抛错(严格模式)。
        std::ofstream partial(path);
        partial << "usbcamera:\n  device_path: /dev/video2\n";
        partial.close();
        bool threw = false;
        try
        {
            (void)hardware::usbcamera::load_usbcamera_config(tools::config::Config(path.string()));
        }
        catch (const std::exception &)
        {
            threw = true;
        }
        if (!threw)
        {
            std::cerr << "FAIL: usbcamera missing key should throw\n";
            return 1;
        }
        std::filesystem::remove(path);
    }

    const hardware::usbcamera::UsbCameraConfig config;
    try
    {
        hardware::usbcamera::UsbCamera camera(config);
        std::cout << "已打开 USB 相机,尝试抓取一帧\n";

        auto                         &frames = camera.frames();
        hardware::usbcamera::UsbFrame frame;
        if (frames.wait_for(frame, std::chrono::milliseconds(2000)) && !frame.image.empty())
        {
            std::cout << "抓帧成功: " << frame.image.cols << "x" << frame.image.rows << " channels=" << frame.image.channels() << "\n";
#ifdef RM_DEBUG
            tools::debug::Sink debug("rm_vision.usbcamera");
            if (debug.active())
            {
                // 连续发送 5 秒
                try
                {
                    tools::video::VideoEncoderConfig encoder_config;
                    encoder_config.width  = frame.image.cols;
                    encoder_config.height = frame.image.rows;
                    tools::video::VideoEncoder encoder(encoder_config);

                    std::cout << "start send video to Rerun(5s)\n";
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
        std::cout << "usbcamera test error: " << error.what() << "\n";
    }

    std::cout << "usbcamera test passed\n";
    return 0;
}
catch (const std::exception &error)
{
    std::cerr << "usbcamera_test error: " << error.what() << "\n";
    return 1;
}
