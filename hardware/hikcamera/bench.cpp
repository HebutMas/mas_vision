// hikcamera 基准自检:分别测「Bayer8 -> BGR 转换」与「实机采集」两类开销。

#include "hardware/hikcamera/hikcamera.hpp"

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cstdlib>
#include <iostream>

namespace
{
constexpr int    WIDTH      = 1440;
constexpr int    HEIGHT     = 1080;
constexpr int    ITERS      = 200;
constexpr double TARGET_FPS = 200.0;

// 重复 ITERS 次转换,返回平均单帧耗时(ms)。输出 Mat 复用,测的是稳态开销。
double bench_convert(const cv::Mat &bayer, int code)
{
    cv::Mat    dst;
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < ITERS; ++i)
    {
        cv::cvtColor(bayer, dst, code);
    }
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count() / ITERS;
}

void report(const char *name, double ms)
{
    std::cout << name << ": " << ms << " ms/frame, " << (1000.0 / ms) << " fps max, " << (ms * TARGET_FPS / 1000.0) << " core @ " << TARGET_FPS
              << "fps\n";
}
} // namespace

int main(int argc, char **argv)
try
{
    const int frames = argc > 1 ? std::atoi(argv[1]) : 400;

    std::cout << "=== Bayer8 -> BGR convert (" << WIDTH << "x" << HEIGHT << ", " << ITERS << " iters) ===\n";
    const cv::Mat bayer(HEIGHT, WIDTH, CV_8UC1, cv::Scalar(128));
    report("bilinear  ", bench_convert(bayer, cv::COLOR_BayerRG2BGR));
    report("edge_aware", bench_convert(bayer, cv::COLOR_BayerRG2BGR_EA));

    std::cout << "\n=== capture (" << frames << " frames) ===\n";
    try
    {
        hardware::hikcamera::HikCamera camera(hardware::hikcamera::HikCameraConfig{});
        hardware::hikcamera::HikFrame  frame;
        auto                          &frames_ref = camera.frames();

        if (!frames_ref.wait_for(frame, std::chrono::milliseconds(2000)))
        {
            std::cout << "no frame within 2s, skip capture bench\n";
            return 0;
        }

        int                            count = 0;
        hardware::hikcamera::TimePoint first = frame.timestamp;
        hardware::hikcamera::TimePoint last  = frame.timestamp;
        const auto                     start = std::chrono::steady_clock::now();
        while (count < frames && frames_ref.wait_for(frame, std::chrono::milliseconds(500)))
        {
            if (count == 0)
            {
                first = frame.timestamp;
            }
            last = frame.timestamp;
            ++count;
        }
        const auto   end  = std::chrono::steady_clock::now();
        const double wall = std::chrono::duration<double>(end - start).count();
        const double span = std::chrono::duration<double>(last - first).count();

        std::cout << "resolution: " << frame.image.cols << "x" << frame.image.rows << " ch=" << frame.image.channels() << "\n";
        std::cout << "wall fps  : " << (count / wall) << " (" << count << " frames / " << wall << " s)\n";
        if (count > 1 && span > 0.0)
        {
            std::cout << "stream fps: " << ((count - 1) / span) << " (by frame timestamps)\n";
        }
    }
    catch (const std::exception &error)
    {
        std::cout << "camera unavailable, skip capture bench: " << error.what() << "\n";
    }

    return 0;
}
catch (const std::exception &error)
{
    std::cerr << "hikcamera_bench error: " << error.what() << "\n";
    return 1;
}
