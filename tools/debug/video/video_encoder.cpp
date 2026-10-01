#include "tools/debug/video/video_encoder.hpp"

#include <cstddef>
#include <opencv2/imgproc.hpp>

#include <cstring>
#include <deque>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
#include <libavutil/opt.h>
}

namespace tools::video
{

namespace
{
// 把 ffmpeg 返回的错误码转成可读信息,便于构造/编码失败时定位。
std::string av_error(int code)
{
    char buffer[AV_ERROR_MAX_STRING_SIZE] = {};
    av_strerror(code, buffer, sizeof(buffer));
    return buffer;
}

// 可核显硬编的 DRM 驱动;i915/xe/amdgpu/radeon
const std::set<std::string> &vaapi_drivers()
{
    static const std::set<std::string> drivers = {"i915", "xe", "amdgpu", "radeon"};
    return drivers;
}

const std::set<std::string> &integrated_drivers()
{
    static const std::set<std::string> drivers = {"i915", "xe"};
    return drivers;
}

// 待尝试的 VAAPI 渲染节点
// configured 为空:扫描 /dev/dri/renderD*,挑出可硬编的核显节点，一个都没识别到走软编
std::vector<std::string> vaapi_devices(const std::string &configured)
{
    if (!configured.empty())
    {
        return {configured};
    }

    std::vector<std::string> integrated;
    std::vector<std::string> others;
    for (int index = 128; index < 192; ++index)
    {
        const std::string node   = "/dev/dri/renderD" + std::to_string(index);
        const std::string driver = "/sys/class/drm/renderD" + std::to_string(index) + "/device/driver";
        std::error_code   error;
        if (!std::filesystem::exists(node, error))
        {
            continue;
        }
        const std::string name = std::filesystem::read_symlink(driver, error).filename().string();
        if (error || vaapi_drivers().count(name) == 0)
        {
            continue;
        }
        (integrated_drivers().count(name) != 0 ? integrated : others).push_back(node);
    }

    std::vector<std::string> nodes;
    nodes.insert(nodes.end(), integrated.begin(), integrated.end());
    nodes.insert(nodes.end(), others.begin(), others.end());
    if (nodes.empty())
    {
        nodes.emplace_back("/dev/dri/renderD128");
    }
    return nodes;
}
} // namespace

struct VideoEncoder::Impl
{
    VideoEncoderConfig                   config;
    bool                                 hardware_ = false; // true=VAAPI 硬编,false=libx264 软编
    const AVCodec                       *codec     = nullptr;
    AVCodecContext                      *ctx       = nullptr;
    AVBufferRef                         *hw_device = nullptr;
    AVBufferRef                         *hw_frames = nullptr;
    AVFrame                             *sw_frame  = nullptr; // 软件 NV12,复用以避免每帧分配
    AVFrame                             *hw_frame  = nullptr; // VAAPI 帧,复用以避免每帧分配
    AVPacket                            *packet    = nullptr;
    cv::Mat                              i420;    // BGR->YUV420 转换的中间结果
    std::deque<std::chrono::nanoseconds> pending; // 已送入编码器、尚未取回的时间戳(FIFO)

    ~Impl()
    {
        if (ctx != nullptr)
        {
            avcodec_free_context(&ctx);
        }
        if (sw_frame != nullptr)
        {
            av_frame_free(&sw_frame);
        }
        if (hw_frame != nullptr)
        {
            av_frame_free(&hw_frame);
        }
        if (packet != nullptr)
        {
            av_packet_free(&packet);
        }
        if (hw_frames != nullptr)
        {
            av_buffer_unref(&hw_frames);
        }
        if (hw_device != nullptr)
        {
            av_buffer_unref(&hw_device);
        }
    }

    // 把 BGR 转成 NV12 写入 sw_frame。
    void convert(const cv::Mat &bgr)
    {
        // 在这段转换里把 OpenCV 限为单线程
        const int saved_threads = cv::getNumThreads();
        if (saved_threads != 1)
        {
            cv::setNumThreads(1);
        }
        cv::cvtColor(bgr, i420, cv::COLOR_BGR2YUV_I420);
        if (saved_threads != 1)
        {
            cv::setNumThreads(saved_threads);
        }

        const int width  = config.width;
        const int height = config.height;

        av_frame_make_writable(sw_frame);
        // Y 平面:逐行拷贝(源行宽 = width,目标行宽可能带对齐)。
        for (int y = 0; y < height; ++y)
        {
            std::memcpy(sw_frame->data[0] + (static_cast<std::ptrdiff_t>(y) * sw_frame->linesize[0]), i420.ptr(y), static_cast<std::size_t>(width));
        }
        // NV12 的 UV 是交错平面;I420 的 U/V 各是 (H/2)×(W/2) 的逻辑平面,按「W 字节/行」的
        // 步长切开:逻辑第 row 行落在 U 块第 (row/2) 行的前半或后半。这样寻址与 i420.step 无关。
        for (int row = 0; row < height / 2; ++row)
        {
            const std::uint8_t *u   = i420.ptr(height + (row / 2)) + (static_cast<std::ptrdiff_t>((row % 2) * (width / 2)));
            const std::uint8_t *v   = i420.ptr(height + (height / 4) + (row / 2)) + (static_cast<std::ptrdiff_t>((row % 2) * (width / 2)));
            std::uint8_t       *dst = sw_frame->data[1] + (static_cast<std::ptrdiff_t>(row) * sw_frame->linesize[1]);
            for (int x = 0; x < width / 2; ++x)
            {
                dst[static_cast<std::ptrdiff_t>(2 * x)] = u[x];
                dst[(2 * x) + 1]                        = v[x];
            }
        }
    }

    // 取出编码器当前所有可用的包。
    std::vector<EncodedFrame> drain(bool flushing)
    {
        std::vector<EncodedFrame> out;
        for (;;)
        {
            const int ret = avcodec_receive_packet(ctx, packet);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            {
                break;
            }
            if (ret < 0)
            {
                if (flushing)
                {
                    break;
                }
                throw std::runtime_error("video encode receive failed: " + av_error(ret));
            }
            EncodedFrame frame;
            frame.data.assign(packet->data, packet->data + packet->size);
            frame.keyframe = (packet->flags & AV_PKT_FLAG_KEY) != 0;
            if (!pending.empty())
            {
                // 包按 pts 递增返回,与送入顺序一致;时间戳用 FIFO 对应。
                frame.timestamp = pending.front();
                pending.pop_front();
            }
            av_packet_unref(packet);
            out.push_back(std::move(frame));
            // 深度 1 时一次 send_frame 最多一个包;多取一轮即可覆盖深度 >1 的流水线。
        }
        return out;
    }

    // 公共编码参数(硬/软编共用)。
    void apply_common(AVCodecContext *context) const
    {
        context->width          = config.width;
        context->height         = config.height;
        context->time_base      = {1, config.fps};
        context->framerate      = {config.fps, 1};
        context->gop_size       = config.gop;
        context->max_b_frames   = 0; // Rerun 的视频流每帧一个时间戳,不接受 B 帧
        context->bit_rate       = config.bitrate;
        context->rc_max_rate    = config.bitrate;
        context->rc_buffer_size = static_cast<int>(config.bitrate); // 1s 缓冲
        context->flags |= AV_CODEC_FLAG_LOW_DELAY;
    }

    [[nodiscard]] AVFrame *alloc_nv12() const
    {
        AVFrame *frame = av_frame_alloc();
        if (frame == nullptr)
        {
            return nullptr;
        }
        frame->format = AV_PIX_FMT_NV12;
        frame->width  = config.width;
        frame->height = config.height;
        if (av_frame_get_buffer(frame, 0) < 0)
        {
            av_frame_free(&frame);
            return nullptr;
        }
        return frame;
    }

    // 尝试在指定 VAAPI 节点上初始化硬编。任何一步失败都返回 false,外层换下一个节点或回退软编。
    bool start_hardware(const std::string &device)
    {
        const AVCodec *hw_codec = avcodec_find_encoder_by_name("h264_vaapi");
        if (hw_codec == nullptr)
        {
            return false;
        }

        AVBufferRef    *dev    = nullptr;
        AVBufferRef    *frames = nullptr;
        AVCodecContext *c      = nullptr;
        AVFrame        *sw     = nullptr;
        AVFrame        *hw     = nullptr;
        AVPacket       *pkt    = nullptr;
        bool            ok     = false;
        do
        {
            if (av_hwdevice_ctx_create(&dev, AV_HWDEVICE_TYPE_VAAPI, device.c_str(), nullptr, 0) < 0)
            {
                break;
            }
            frames = av_hwframe_ctx_alloc(dev);
            if (frames == nullptr)
            {
                break;
            }
            auto *frames_ctx              = reinterpret_cast<AVHWFramesContext *>(frames->data);
            frames_ctx->format            = AV_PIX_FMT_VAAPI;
            frames_ctx->sw_format         = AV_PIX_FMT_NV12;
            frames_ctx->width             = config.width;
            frames_ctx->height            = config.height;
            frames_ctx->initial_pool_size = 64; // 足够覆盖 async_depth 对应的在途帧
            if (av_hwframe_ctx_init(frames) < 0)
            {
                break;
            }

            c = avcodec_alloc_context3(hw_codec);
            if (c == nullptr)
            {
                break;
            }
            apply_common(c);
            c->pix_fmt      = AV_PIX_FMT_VAAPI;
            c->thread_count = 1;
            c->thread_type  = 0;
            if (av_opt_set(c->priv_data, "rc_mode", "CBR", 0) < 0)
            {
                break;
            }
            av_opt_set_int(c->priv_data, "async_depth", config.async_depth, 0);
            c->hw_frames_ctx = av_buffer_ref(frames);
            if (c->hw_frames_ctx == nullptr || avcodec_open2(c, hw_codec, nullptr) < 0)
            {
                break;
            }

            sw = alloc_nv12();
            if (sw == nullptr)
            {
                break;
            }
            hw = av_frame_alloc();
            if (hw == nullptr)
            {
                break;
            }
            pkt = av_packet_alloc();
            if (pkt == nullptr)
            {
                break;
            }
            ok = true;
        } while (false);

        if (!ok)
        {
            av_packet_free(&pkt);
            av_frame_free(&hw);
            av_frame_free(&sw);
            avcodec_free_context(&c);
            av_buffer_unref(&frames);
            av_buffer_unref(&dev);
            return false;
        }

        codec     = hw_codec;
        ctx       = c;
        hw_device = dev;
        hw_frames = frames;
        sw_frame  = sw;
        hw_frame  = hw;
        packet    = pkt;
        hardware_ = true;
        return true;
    }

    // CPU 软编(libx264)兜底。
    void start_software()
    {
        codec = avcodec_find_encoder_by_name("libx264");
        if (codec == nullptr)
        {
            throw std::runtime_error("video encoder: no VAAPI device and libx264 not available in ffmpeg build");
        }
        ctx = avcodec_alloc_context3(codec);
        if (ctx == nullptr)
        {
            throw std::runtime_error("video encoder: avcodec_alloc_context3 failed");
        }
        apply_common(ctx);
        ctx->pix_fmt      = AV_PIX_FMT_NV12; // convert() 产出 NV12,libx264 支持
        ctx->thread_count = 0;               // 软编不限线程,尽量吃满 CPU
        if (avcodec_open2(ctx, codec, nullptr) < 0)
        {
            throw std::runtime_error("video encoder: libx264 open failed");
        }

        sw_frame = alloc_nv12();
        if (sw_frame == nullptr)
        {
            throw std::runtime_error("video encoder: allocate NV12 frame failed");
        }
        packet = av_packet_alloc();
        if (packet == nullptr)
        {
            throw std::runtime_error("video encoder: av_packet_alloc failed");
        }
    }
};

VideoEncoder::VideoEncoder(const VideoEncoderConfig &config) : impl_(std::make_unique<Impl>())
{
    impl_->config = config;

    // 核显优先;逐个候选节点尝试硬编,全失败则回退软编。
    for (const std::string &device : vaapi_devices(config.device))
    {
        if (impl_->start_hardware(device))
        {
            return;
        }
    }
    impl_->start_software();
}

VideoEncoder::~VideoEncoder() = default;

bool VideoEncoder::hardware() const { return impl_->hardware_; }

std::vector<EncodedFrame> VideoEncoder::encode(const cv::Mat &bgr, std::chrono::nanoseconds timestamp)
{
    Impl &impl = *impl_;
    if (bgr.empty() || bgr.type() != CV_8UC3 || bgr.cols != impl.config.width || bgr.rows != impl.config.height)
    {
        throw std::runtime_error("video encoder: frame must be " + std::to_string(impl.config.width) + "x" + std::to_string(impl.config.height) +
                                 " 8UC3 BGR");
    }

    impl.convert(bgr);

    AVFrame *frame = impl.sw_frame;
    if (impl.hardware_)
    {
        frame = impl.hw_frame;
        av_frame_unref(frame);
        int ret = av_hwframe_get_buffer(impl.hw_frames, frame, 0);
        if (ret < 0)
        {
            throw std::runtime_error("video encoder: av_hwframe_get_buffer failed: " + av_error(ret));
        }
        ret = av_hwframe_transfer_data(frame, impl.sw_frame, 0);
        if (ret < 0)
        {
            throw std::runtime_error("video encoder: upload to VAAPI failed: " + av_error(ret));
        }
    }
    frame->pts = static_cast<std::int64_t>(impl.pending.size());
    impl.pending.push_back(timestamp);

    const int ret = avcodec_send_frame(impl.ctx, frame);
    if (ret < 0)
    {
        impl.pending.pop_back();
        throw std::runtime_error("video encoder: avcodec_send_frame failed: " + av_error(ret));
    }

    return impl.drain(false);
}

std::vector<EncodedFrame> VideoEncoder::flush()
{
    int const ret = avcodec_send_frame(impl_->ctx, nullptr);
    if (ret < 0 && ret != AVERROR_EOF)
    {
        throw std::runtime_error("video encoder: flush failed: " + av_error(ret));
    }
    return impl_->drain(true);
}

} // namespace tools::video
