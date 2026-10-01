#include "tools/debug/video/video_encoder.hpp"

#include <cstddef>
#include <opencv2/core.hpp>

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <vector>

#include <unistd.h>

namespace
{

// 在 Annex B 码流里找指定类型的 NAL(H.264:type = 字节 & 0x1F)。
// [[maybe_unused]]:仅在 assert 中调用,Release(NDEBUG)下 assert 被裁剪,否则会报未使用。
[[maybe_unused]] bool has_nal(const std::vector<std::uint8_t> &data, int type)
{
    for (std::size_t i = 0; i + 3 <= data.size();)
    {
        std::size_t start = 0;
        if (i + 4 <= data.size() && data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 0 && data[i + 3] == 1)
        {
            start = i + 4;
        }
        else if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1)
        {
            start = i + 3;
        }
        else
        {
            ++i;
            continue;
        }
        if (start < data.size() && (data[start] & 0x1F) == type)
        {
            return true;
        }
        i = start;
    }
    return false;
}

} // namespace

int main()
{
    const char *device = "/dev/dri/renderD128";
    if (::access(device, F_OK) != 0)
    {
        std::cout << "video test skipped: " << device << " not present\n";
        return 0;
    }

    tools::video::VideoEncoderConfig config;
    config.width   = 640;
    config.height  = 480;
    config.fps     = 50;
    config.bitrate = 4'000'000;
    config.gop     = 50;

    tools::video::VideoEncoder encoder(config);

    constexpr int FRAMES_COUNT = 100;
    const auto    period       = std::chrono::nanoseconds(1'000'000'000LL / config.fps);

    std::vector<tools::video::EncodedFrame> encoded;
    cv::Mat                                 image(config.height, config.width, CV_8UC3);
    for (int i = 0; i < FRAMES_COUNT; ++i)
    {
        // 带纹理的合成画面,避免编码器在纯色上把码率压得没有参考价值。
        for (int y = 0; y < image.rows; ++y)
        {
            auto *row = image.ptr<std::uint8_t>(y);
            for (int x = 0; x < image.cols; ++x)
            {
                row[static_cast<std::ptrdiff_t>(3 * x)] = static_cast<std::uint8_t>(x);
                row[(3 * x) + 1]                        = static_cast<std::uint8_t>(y);
                row[(3 * x) + 2]                        = static_cast<std::uint8_t>(x + y + i);
            }
        }
        auto packet = encoder.encode(image, period * i);
        for (auto &frame : packet)
        {
            encoded.push_back(std::move(frame));
        }
    }
    for (auto &frame : encoder.flush())
    {
        encoded.push_back(std::move(frame));
    }

    // 每帧恰好一个访问单元(编码器不做 B 帧/多 slice)。
    assert(encoded.size() == static_cast<std::size_t>(FRAMES_COUNT));

    int         keyframes   = 0;
    std::size_t total_bytes = 0;
    // NOLINTNEXTLINE(modernize-loop-convert): 循环体 assert 用到下标 i(Release 下 assert 被裁剪)。
    for (std::size_t i = 0; i < encoded.size(); ++i)
    {
        const tools::video::EncodedFrame &frame = encoded[i];
        assert(!frame.data.empty());
        // 时间戳按送入顺序对应。
        assert(frame.timestamp == period * static_cast<std::int64_t>(i));
        total_bytes += frame.data.size();
        if (frame.keyframe)
        {
            ++keyframes;
            // 关键帧必须是 IDR 且自带 SPS,否则 Rerun 的视频流无法从它开始解码。
            assert(has_nal(frame.data, 5)); // IDR
            assert(has_nal(frame.data, 7)); // SPS
        }
        else
        {
            assert(!has_nal(frame.data, 7)); // 非关键帧不带 SPS
        }
    }

    assert(encoded.front().keyframe);
    assert(keyframes >= FRAMES_COUNT / config.gop); // gop=50 => 至少 2 个

    // 码率受 CBR 控制:整体落在目标码率的一半到两倍之间。
    const double seconds   = static_cast<double>(FRAMES_COUNT) / config.fps;
    const double real_rate = static_cast<double>(total_bytes) * 8.0 / seconds;
    assert(real_rate > 0.5 * config.bitrate && real_rate < 3.0 * config.bitrate);

    std::cout << "video test passed: " << encoded.size() << " frames, " << keyframes << " keyframes, " << (real_rate / 1'000'000.0)
              << " Mbps (target " << (config.bitrate / 1'000'000.0) << " Mbps), path=" << (encoder.hardware() ? "VAAPI" : "CPU") << "\n";

    // 软编测试:指定一个不存在的 VAAPI 节点,应回退 libx264 且仍能出流。
    {
        tools::video::VideoEncoderConfig soft_config = config;
        soft_config.device                           = "/dev/dri/renderD_nonexistent";
        tools::video::VideoEncoder soft_encoder(soft_config);
        assert(!soft_encoder.hardware());

        std::vector<tools::video::EncodedFrame> soft;
        for (auto &frame : soft_encoder.encode(image, std::chrono::nanoseconds(0)))
        {
            soft.push_back(std::move(frame));
        }
        for (auto &frame : soft_encoder.flush())
        {
            soft.push_back(std::move(frame));
        }
        assert(!soft.empty());
        std::cout << "software fallback test passed: " << soft.size() << " frames\n";
    }

    // 可选:导出码流,便于用 `ffmpeg -f h264 -i <file>` 复核画面/色彩是否正确。
    if (const char *dump = std::getenv("RM_VIDEO_DUMP"); dump != nullptr)
    {
        std::FILE *file = std::fopen(dump, "wb");
        assert(file != nullptr);
        for (const auto &frame : encoded)
        {
            std::fwrite(frame.data.data(), 1, frame.data.size(), file);
        }
        std::fclose(file);
        std::cout << "dumped bitstream to " << dump << "\n";
    }
    return 0;
}
