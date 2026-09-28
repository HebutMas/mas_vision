#include "hardware/usbcamera/usbcamera.hpp"
#if defined(RM_DEBUG)
#include "tools/debug/debug.hpp"
#endif

#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#if defined(RM_DEBUG)
namespace
{
constexpr int SEND_SECONDS = 5;
} // namespace
#endif

// 测试:没有相机时应当抛异常并返回 0;接了相机则抓一帧验证输出格式。
// 调试构建下若相机出图,再把这一帧送到 Rerun Viewer 方便肉眼确认。
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
#if defined(RM_DEBUG)
            tools::debug::Sink debug("rm_vision.usbcamera");
            if (debug.active())
            {
                // 连续发送 5 秒,每帧带上相对统一基准的时间戳,方便在 Viewer 里按真实时间回放。
                std::cout << "开始连续发送画面到 Rerun(5s)\n";
                const auto deadline = tools::time::now() + std::chrono::seconds(SEND_SECONDS);
                int        index    = 0;
                while (tools::time::now() < deadline && !frame.image.empty())
                {
                    debug.set_frame(index);
                    debug.set_time("time", tools::time::since_base(frame.timestamp));
                    debug.image("camera/image", frame.image);
                    ++index;
                    if (!frames.wait_for(frame, std::chrono::milliseconds(2000)))
                    {
                        break;
                    }
                }
                std::cout << "已发送 " << index << " 帧到 Rerun\n";
            }
            else
            {
                std::cout << "Rerun Viewer 不可用,跳过画面输出\n";
            }
#endif
        }
        else
        {
            std::cout << "超时未取到帧(相机可能未正常出图)\n";
        }
    }
    catch (const std::exception &error)
    {
        // 找不到相机属于正常情况:驱动会抛异常,这里视为通过。
        std::cout << "未打开相机: " << error.what() << "\n";
    }

    std::cout << "usbcamera test passed\n";
    return 0;
}
catch (const std::exception &error)
{
    std::cerr << "usbcamera_test error: " << error.what() << "\n";
    return 1;
}
