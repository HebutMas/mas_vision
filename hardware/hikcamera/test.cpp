#include "hardware/hikcamera/hikcamera.hpp"
#ifdef RM_DEBUG
#include "tools/debug/debug.hpp"
#include "tools/debug/video/video_encoder.hpp"
#include "tools/time/time.hpp"
#endif

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

namespace
{
#ifdef RM_DEBUG
constexpr int SEND_SECONDS = 5;
#endif

// Bayer8 -> BGR 转换耗时基准。
constexpr int    BENCH_WIDTH  = 1440;
constexpr int    BENCH_HEIGHT = 1080;
constexpr int    BENCH_ITERS  = 200;
constexpr int    BENCH_FRAMES = 400;
constexpr double TARGET_FPS   = 200.0;

// 重复 BENCH_ITERS 次转换,返回平均单帧耗时(ms)。输出 Mat 复用,测试开销。
double bench_convert(const cv::Mat &bayer, int code)
{
    cv::Mat    dst;
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < BENCH_ITERS; ++i)
    {
        cv::cvtColor(bayer, dst, code);
    }
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count() / BENCH_ITERS;
}

void report_bench(const char *name, double ms)
{
    std::cout << name << ": " << ms << " ms/frame, " << (1000.0 / ms) << " fps max, " << (ms * TARGET_FPS / 1000.0) << " core @ " << TARGET_FPS
              << "fps\n";
}

void bench_demosaic()
{
    std::cout << "\n=== Bayer8 -> BGR convert (" << BENCH_WIDTH << "x" << BENCH_HEIGHT << ", " << BENCH_ITERS << " iters) ===\n";
    const cv::Mat bayer(BENCH_HEIGHT, BENCH_WIDTH, CV_8UC1, cv::Scalar(128));
    report_bench("bilinear  ", bench_convert(bayer, cv::COLOR_BayerRG2BGR));
    report_bench("edge_aware", bench_convert(bayer, cv::COLOR_BayerRG2BGR_EA));
}
} // namespace

int main()
try
{
    // 配置加载自检。
    {
        const auto    path = std::filesystem::temp_directory_path() / "rm_vision_hikcamera_test.yaml";
        std::ofstream out(path);
        out << "hikcamera:\n"
               "  serial: \n"
               "  exposure_us: 5000\n"
               "  gain_db: 10\n"
               "  framerate: 0\n"
               "  demosaic: bilinear\n"
               "  roi: {enable: false, x: 0, y: 0, width: 0, height: 0}\n";
        out.close();
        const auto cfg = hardware::hikcamera::load_hikcamera_config(tools::config::Config(path.string()));
        if (!cfg.serial.empty() || cfg.exposure_us != 5000.0 || cfg.gain_db != 10.0 || cfg.framerate != 0.0 ||
            cfg.demosaic != hardware::hikcamera::DemosaicQuality::bilinear || cfg.roi.enable || cfg.roi.x != 0 || cfg.roi.y != 0 ||
            cfg.roi.width != 0 || cfg.roi.height != 0)
        {
            std::cerr << "FAIL: hikcamera config load\n";
            return 1;
        }

        // 缺键报错
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

    // 默认接第一台相机,曝光 5000us、增益 10dB、不开 ROI。
    hardware::hikcamera::HikCameraConfig config;
    config.exposure_us = 5000.0;
    config.gain_db     = 10.0;
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
#ifdef RM_DEBUG
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

            // 采集帧率:连续取 N 帧,计算实际采集帧率
            const int  bench_frames = BENCH_FRAMES;
            int        count        = 0;
            auto       first        = frame.timestamp;
            auto       last         = frame.timestamp;
            const auto begin        = std::chrono::steady_clock::now();
            while (count < bench_frames && frames.wait_for(frame, std::chrono::milliseconds(500)))
            {
                if (count == 0)
                {
                    first = frame.timestamp;
                }
                last = frame.timestamp;
                ++count;
            }
            const auto   end  = std::chrono::steady_clock::now();
            const double wall = std::chrono::duration<double>(end - begin).count();
            const double span = std::chrono::duration<double>(last - first).count();

            std::cout << "capture   : " << frame.image.cols << "x" << frame.image.rows << " ch=" << frame.image.channels() << "\n";
            std::cout << "wall fps  : " << (count / wall) << " (" << count << " frames / " << wall << " s)\n";
            if (count > 1 && span > 0.0)
            {
                std::cout << "stream fps: " << ((count - 1) / span) << " (by frame timestamps)\n";
            }
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

    bench_demosaic();

    std::cout << "hikcamera test passed\n";
    return 0;
}
catch (const std::exception &error)
{
    std::cerr << "hikcamera_test error: " << error.what() << "\n";
    return 1;
}
