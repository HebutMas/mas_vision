#include "tools/debug/video/video_encoder.hpp"

#include <cstddef>
#include <opencv2/imgproc.hpp>

#include <cstring>
#include <deque>
#include <stdexcept>
#include <utility>

#include <algorithm>
#include <filesystem>
#include <string>
#include <system_error>
#include <unistd.h>
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

// 这些内核驱动背后有可用的 VAAPI 用户态驱动:i915 走 iHD、amdgpu 走 Mesa、xe 是新版
// Intel 驱动。NVIDIA 没有 VAAPI 后端,所以 nvidia 节点必须排除。
bool supports_vaapi(const std::string & driver)
{
  return driver == "i915" || driver == "amdgpu" || driver == "xe" || driver == "radeon";
}

// "renderD128" -> 128;不是渲染节点返回 -1。
int render_node_index(const std::string & name)
{
  if (name.rfind("renderD", 0) != 0)
  {
    return -1;
  }
  try
  {
    return std::stoi(name.substr(7));
  }
  catch (const std::exception &)
  {
    return -1;
  }
}

// 某个目录下所有渲染节点的 <编号, 路径>,按编号升序。目录不存在时返回空。
std::vector<std::pair<int, std::string>> render_nodes(const std::string & directory)
{
  std::vector<std::pair<int, std::string>> nodes;
  std::error_code                          error;
  for (const auto & entry : std::filesystem::directory_iterator(directory, error))
  {
    const std::string name  = entry.path().filename().string();
    const int         index = render_node_index(name);
    if (index >= 0)
    {
      nodes.emplace_back(index, entry.path().string());
    }
  }
  std::sort(nodes.begin(), nodes.end(),
            [](const auto & lhs, const auto & rhs) { return lhs.first < rhs.first; });
  return nodes;
}
} // namespace

std::string find_vaapi_device()
{
  // 1. 按 sysfs 里绑定的内核驱动,挑出有能力做 VAAPI 的节点。
  std::vector<std::pair<int, std::string>> supported;
  for (const auto & [index, sysfs_path] : render_nodes("/sys/class/drm"))
  {
    std::error_code   error;
    const auto        driver_link = std::filesystem::read_symlink(sysfs_path + "/device/driver", error);
    const std::string driver      = driver_link.filename().string();
    if (!error && supports_vaapi(driver))
    {
      supported.emplace_back(index, "/dev/dri/renderD" + std::to_string(index));
    }
  }

  // 2. 优先选 /dev 节点确实存在的那个。容器 / sandbox 里 /dev/dri 可能没挂进来,这时仍
  //    返回探测到的节点,好让报错信息指向正确的设备,而不是编号最小的那个。
  for (const auto & candidate : supported)
  {
    if (::access(candidate.second.c_str(), F_OK) == 0)
    {
      return candidate.second;
    }
  }
  if (!supported.empty())
  {
    return supported.front().second;
  }

  // 3. 驱动认不出来(内核模块改名等):退回第一个存在的渲染节点,再不行用历史默认值。
  const auto present = render_nodes("/dev/dri");
  if (!present.empty())
  {
    return present.front().second;
  }
  return "/dev/dri/renderD128";
}

struct VideoEncoder::Impl
{
  VideoEncoderConfig            config;
  const AVCodec *               codec        = nullptr;
  AVCodecContext *              ctx          = nullptr;
  AVBufferRef *                 hw_device    = nullptr;
  AVBufferRef *                 hw_frames    = nullptr;
  AVFrame *                     sw_frame     = nullptr; // 软件 NV12,复用以避免每帧分配
  AVFrame *                     hw_frame     = nullptr; // VAAPI 帧,复用以避免每帧分配
  AVPacket *                    packet       = nullptr;
  cv::Mat                       i420;                 // BGR->YUV420 转换的中间结果
  std::deque<std::chrono::nanoseconds> pending;       // 已送入编码器、尚未取回的时间戳(FIFO)

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
  void convert(const cv::Mat & bgr)
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
      std::memcpy(sw_frame->data[0] + (static_cast<std::ptrdiff_t>(y) * sw_frame->linesize[0]),
                  i420.ptr(y), static_cast<std::size_t>(width));
    }
    // NV12 的 UV 是交错平面;I420 的 U/V 各是 (H/2)×(W/2) 的逻辑平面,按「W 字节/行」的
    // 步长切开:逻辑第 row 行落在 U 块第 (row/2) 行的前半或后半。这样寻址与 i420.step 无关。
    for (int row = 0; row < height / 2; ++row)
    {
      const std::uint8_t * u   = i420.ptr(height + (row / 2)) + (static_cast<std::ptrdiff_t>((row % 2) * (width / 2)));
      const std::uint8_t * v   = i420.ptr(height + (height / 4) + (row / 2)) + (static_cast<std::ptrdiff_t>((row % 2) * (width / 2)));
      std::uint8_t *       dst = sw_frame->data[1] + (static_cast<std::ptrdiff_t>(row) * sw_frame->linesize[1]);
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
};

VideoEncoder::VideoEncoder(const VideoEncoderConfig & config) : impl_(std::make_unique<Impl>())
{
  impl_->config = config;
  // 留空表示自动探测:双显卡机器上 renderD128 可能是没有 VAAPI 后端的 NVIDIA 节点。
  if (impl_->config.device.empty())
  {
    impl_->config.device = find_vaapi_device();
  }

  impl_->codec = avcodec_find_encoder_by_name("h264_vaapi");
  if (impl_->codec == nullptr)
  {
    throw std::runtime_error("video encoder: h264_vaapi not available in ffmpeg build");
  }

  impl_->ctx = avcodec_alloc_context3(impl_->codec);
  if (impl_->ctx == nullptr)
  {
    throw std::runtime_error("video encoder: avcodec_alloc_context3 failed");
  }

  AVCodecContext * ctx = impl_->ctx;
  ctx->width        = config.width;
  ctx->height       = config.height;
  ctx->time_base    = {1, config.fps};
  ctx->framerate    = {config.fps, 1};
  ctx->pix_fmt      = AV_PIX_FMT_VAAPI;
  ctx->gop_size     = config.gop;
  ctx->max_b_frames = 0; // Rerun 的视频流每帧一个时间戳,不接受 B 帧
  ctx->bit_rate     = config.bitrate;
  ctx->rc_max_rate  = config.bitrate;
  ctx->rc_buffer_size = static_cast<int>(config.bitrate); // 1s 缓冲
  ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;
  ctx->thread_count = 1;
  ctx->thread_type  = 0;

  if (av_opt_set(ctx->priv_data, "rc_mode", "CBR", 0) < 0)
  {
    throw std::runtime_error("video encoder: failed to set rc_mode=CBR");
  }
  av_opt_set_int(ctx->priv_data, "async_depth", config.async_depth, 0);

  int ret = av_hwdevice_ctx_create(&impl_->hw_device, AV_HWDEVICE_TYPE_VAAPI,
                                   impl_->config.device.c_str(), nullptr, 0);
  if (ret < 0)
  {
    throw std::runtime_error("video encoder: VAAPI device " + impl_->config.device + " failed: " +
                             av_error(ret));
  }

  impl_->hw_frames = av_hwframe_ctx_alloc(impl_->hw_device);
  if (impl_->hw_frames == nullptr)
  {
    throw std::runtime_error("video encoder: av_hwframe_ctx_alloc failed");
  }
  auto * frames            = reinterpret_cast<AVHWFramesContext *>(impl_->hw_frames->data);
  frames->format           = AV_PIX_FMT_VAAPI;
  frames->sw_format        = AV_PIX_FMT_NV12;
  frames->width            = config.width;
  frames->height           = config.height;
  frames->initial_pool_size = 64; // 足够覆盖 async_depth 对应的在途帧
  ret = av_hwframe_ctx_init(impl_->hw_frames);
  if (ret < 0)
  {
    throw std::runtime_error("video encoder: av_hwframe_ctx_init failed: " + av_error(ret));
  }
  ctx->hw_frames_ctx = av_buffer_ref(impl_->hw_frames);

  ret = avcodec_open2(ctx, impl_->codec, nullptr);
  if (ret < 0)
  {
    throw std::runtime_error("video encoder: avcodec_open2 failed: " + av_error(ret));
  }

  impl_->sw_frame         = av_frame_alloc();
  impl_->sw_frame->format = AV_PIX_FMT_NV12;
  impl_->sw_frame->width  = config.width;
  impl_->sw_frame->height = config.height;
  if (impl_->sw_frame == nullptr || av_frame_get_buffer(impl_->sw_frame, 0) < 0)
  {
    throw std::runtime_error("video encoder: allocate NV12 frame failed");
  }

  impl_->hw_frame = av_frame_alloc();
  if (impl_->hw_frame == nullptr)
  {
    throw std::runtime_error("video encoder: av_frame_alloc (hw) failed");
  }

  impl_->packet = av_packet_alloc();
  if (impl_->packet == nullptr)
  {
    throw std::runtime_error("video encoder: av_packet_alloc failed");
  }
}

VideoEncoder::~VideoEncoder() = default;

std::vector<EncodedFrame> VideoEncoder::encode(const cv::Mat & bgr, std::chrono::nanoseconds timestamp)
{
  Impl & impl = *impl_;
  if (bgr.empty() || bgr.type() != CV_8UC3 || bgr.cols != impl.config.width ||
      bgr.rows != impl.config.height)
  {
    throw std::runtime_error("video encoder: frame must be " +
                             std::to_string(impl.config.width) + "x" +
                             std::to_string(impl.config.height) + " 8UC3 BGR");
  }

  impl.convert(bgr);

  AVFrame * hw_frame = impl.hw_frame;
  av_frame_unref(hw_frame);
  int ret = av_hwframe_get_buffer(impl.hw_frames, hw_frame, 0);
  if (ret < 0)
  {
    throw std::runtime_error("video encoder: av_hwframe_get_buffer failed: " + av_error(ret));
  }
  ret = av_hwframe_transfer_data(hw_frame, impl.sw_frame, 0);
  if (ret < 0)
  {
    throw std::runtime_error("video encoder: upload to VAAPI failed: " + av_error(ret));
  }
  hw_frame->pts = static_cast<std::int64_t>(impl.pending.size());
  impl.pending.push_back(timestamp);

  ret = avcodec_send_frame(impl.ctx, hw_frame);
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
