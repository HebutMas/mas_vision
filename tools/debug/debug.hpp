#pragma once

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#if defined(RM_DEBUG)
// 仅调试构建才引入 Rerun SDK 与网络探测所需的系统头文件。
#include <rerun.hpp>

#include <cerrno>
#include <fcntl.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace tools::debug
{

// Viewer 的默认 gRPC 地址。端口 9876 是 Rerun 的默认值。
inline constexpr const char *DEFAULT_ADDRESS = "rerun+http://127.0.0.1:9876/proxy";

// 日志等级,映射到 Rerun 的 TextLogLevel。
enum class Level : std::uint8_t
{
    debug,
    info,
    warn,
    error
};

class Sink;

// 进程内的日志出口:Sink 构造时登记、析构时注销,供自由函数 log() 转发到 Rerun。
namespace detail
{
inline Sink *&global_sink()
{
    static Sink *sink = nullptr;
    return sink;
}

// 统一的控制台格式:debug/info 走 stdout,warn/error 走 stderr。
inline void print_console(Level level, const std::string &message)
{
    std::ostream &out = (level == Level::warn || level == Level::error) ? std::cerr : std::cout;
    out << message << '\n' << std::flush;
}
} // namespace detail

// 统一日志出口:始终打印到控制台(debug/info -> stdout,warn/error -> stderr),
// 进程内存在活动 Sink 时同步转发到 Rerun(path 是 Rerun 里的实体路径)。
inline void log(Level level, const std::string &message, const std::string &path = "log");

#if defined(RM_DEBUG)
// ---------------------------------------------------------------------------
// 内部实现细节:地址解析 + 连通性探测。只在调试构建里存在。
// ---------------------------------------------------------------------------
namespace detail
{

// Level -> Rerun 等级。
inline rerun::TextLogLevel to_rerun_level(Level level)
{
    switch (level)
    {
    case Level::debug:
        return rerun::TextLogLevel::Debug;
    case Level::warn:
        return rerun::TextLogLevel::Warning;
    case Level::error:
        return rerun::TextLogLevel::Error;
    case Level::info:
        break;
    }
    return rerun::TextLogLevel::Info;
}

// 从 "rerun+http://host:port/proxy" 中解析出 host 与 port。
// 只处理 SDK 文档允许的 rerun:// / rerun+http:// / rerun+https:// 形式。
inline bool parse_address(const std::string &address, std::string &host, uint16_t &port)
{
    // 跳过 "scheme://";若没有 scheme 则整串视为 "host:port/path"。
    const std::string scheme     = "://";
    const auto        scheme_pos = address.find(scheme);
    const std::string rest       = address.substr(scheme_pos == std::string::npos ? 0 : scheme_pos + scheme.size());

    // 冒号前是 host,冒号后到下一个 '/' 之前是 port。
    const auto colon = rest.find(':');
    if (colon == std::string::npos)
    {
        return false;
    }
    host                       = rest.substr(0, colon);
    const auto        slash    = rest.find('/', colon);
    const std::string port_str = rest.substr(colon + 1, slash == std::string::npos ? std::string::npos : slash - colon - 1);

    try
    {
        port = static_cast<uint16_t>(std::stoi(port_str));
    }
    catch (...)
    {
        return false;
    }
    return !host.empty();
}

// 用一次「非阻塞 TCP 连接」判断 Viewer 是否在线。
// Rerun 的 gRPC sink 在连不上时会等待约 5
// 秒,并且进程退出时可能卡在关闭阶段。机器人主循环不能接受这种阻塞, 所以先快速探一下(默认
// 300ms);探不到就直接不启用调试。
inline bool viewer_online(const std::string &address, int timeout_ms = 300)
{
    std::string host;
    uint16_t    port = 0;
    if (!parse_address(address, host, port))
    {
        return false;
    }

    addrinfo hints{};
    hints.ai_family    = AF_UNSPEC;
    hints.ai_socktype  = SOCK_STREAM;
    addrinfo *resolved = nullptr;
    if (::getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &resolved) != 0)
    {
        return false;
    }

    // 依次尝试解析出的每个地址(IPv4/IPv6),任意一个连上即视为在线。
    bool online = false;
    for (addrinfo *candidate = resolved; candidate != nullptr && !online; candidate = candidate->ai_next)
    {
        const int fd = ::socket(candidate->ai_family, candidate->ai_socktype, candidate->ai_protocol);
        if (fd < 0)
        {
            continue;
        }
        const int flags = ::fcntl(fd, F_GETFL, 0);
        ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        int rc = ::connect(fd, candidate->ai_addr, candidate->ai_addrlen);
        if (rc == 0)
        {
            online = true; // 立即连上(本机回环常见)。
        }
        else if (errno == EINPROGRESS)
        {
            // 连接进行中:用 select 等它变为可写,再用 SO_ERROR 确认是否真的成功。
            fd_set writable;
            FD_ZERO(&writable);
            FD_SET(fd, &writable);
            timeval timeout{timeout_ms / 1000, static_cast<long>(timeout_ms % 1000) * 1000};
            if (::select(fd + 1, nullptr, &writable, nullptr, &timeout) > 0)
            {
                int       error  = 0;
                socklen_t length = sizeof(error);
                ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &length);
                online = (error == 0);
            }
        }
        ::close(fd);
    }

    ::freeaddrinfo(resolved);
    return online;
}

} // namespace detail
#endif // RM_DEBUG

// Sink:调试数据出口。一个进程通常只建一个,生命周期覆盖整个主循环。
class Sink
{
  public:
    // application_id 用于在 Viewer 里区分不同程序,建议用 "rm_vision.<兵种>"。
    // address 留空则用 DEFAULT_ADDRESS。
    Sink(const std::string &application_id, const std::string &address = DEFAULT_ADDRESS)
    {
#if defined(RM_DEBUG)
        if (detail::viewer_online(address))
        {
            stream_ = std::make_unique<rerun::RecordingStream>(application_id);
            active_ = stream_->connect_grpc(address).is_ok();
        }
        if (active_)
        {
            detail::global_sink() = this;
        }
#else
        (void)application_id;
        (void)address;
#endif
    }

    ~Sink()
    {
        if (detail::global_sink() == this)
        {
            detail::global_sink() = nullptr;
        }
    }

    Sink(const Sink &)            = delete;
    Sink &operator=(const Sink &) = delete;

    // Viewer 是否可用。调用方一般不需要判断,直接调用各接口即可。
    [[nodiscard]] bool active() const { return active_; }

    // 设置帧序号时间轴。Viewer 里可按帧回放 / 逐帧对比,建议在主循环开头调用。
    void set_frame(int64_t frame)
    {
#if defined(RM_DEBUG)
        if (!active_)
        {
            return;
        }
        stream_->set_time_sequence("frame", frame);
#else
        (void)frame;
#endif
    }

    // 设置时间轴为「从采集起点到当前帧的时长」,配合相机时间戳实现按真实时间回放。
    void set_time(const std::string &timeline, std::chrono::nanoseconds since_start)
    {
#if defined(RM_DEBUG)
        if (!active_)
        {
            return;
        }
        stream_->set_time_duration(timeline, since_start);
#else
        (void)timeline;
        (void)since_start;
#endif
    }

    // 发送单个标量,Viewer 中表现为一条曲线(如 ekf/yaw、debug/latency)。
    void data(const std::string &path, double value)
    {
#if defined(RM_DEBUG)
        if (!active_)
        {
            return;
        }
        stream_->log(path, rerun::Scalars(rerun::Scalar(value)));
#else
        (void)path;
        (void)value;
#endif
    }

    // 发送多分量数据到同一张曲线图:整体记到 <path> 一个 entity,Viewer 里是同一图内的多条 series
    // (分量数须逐帧一致,如四元数 4 条)。
    void data(const std::string &path, const std::vector<double> &values)
    {
#if defined(RM_DEBUG)
        if (!active_ || values.empty())
        {
            return;
        }
        stream_->log(path, rerun::Scalars(values));
#else
        (void)path;
        (void)values;
#endif
    }

    // 发送一帧视频样本
    void video(const std::string &path, std::vector<std::uint8_t> data, bool keyframe)
    {
#if defined(RM_DEBUG)
        if (!active_ || data.empty())
        {
            return;
        }
        if (video_codec_paths_.insert(path).second)
        {
            stream_->log_static(path, rerun::archetypes::VideoStream().with_codec(rerun::components::VideoCodec::H264));
        }
        rerun::components::VideoSample sample(rerun::Collection<std::uint8_t>::take_ownership(std::move(data)));
        stream_->log(path, rerun::archetypes::VideoStream().with_sample(sample).with_is_keyframe(keyframe));
#else
        (void)path;
        (void)data;
        (void)keyframe;
#endif
    }

    // 发送一张图片（jpeg格式）
    void image(const std::string &path, std::vector<std::uint8_t> jpeg)
    {
#if defined(RM_DEBUG)
        if (!active_ || jpeg.empty())
        {
            return;
        }
        stream_->log(path, rerun::archetypes::EncodedImage::from_bytes(rerun::Collection<std::uint8_t>::take_ownership(std::move(jpeg)),
                                                                       rerun::components::MediaType::jpeg()));
#else
        (void)path;
        (void)jpeg;
#endif
    }

    // 始终打印到控制台,Viewer 可用时同时发往 Rerun,path 是 Rerun 里的实体路径;控制台忽略它。
    void log(Level level, const std::string &message, const std::string &path = "log")
    {
        detail::print_console(level, message);
#if defined(RM_DEBUG)
        if (!active_)
        {
            return;
        }
        stream_->log(path, rerun::TextLog(message).with_level(detail::to_rerun_level(level)));
#else
        (void)path;
#endif
    }

  private:
    // 调试关闭时也会有一个恒为 false 的成员,保证所有方法里的分支写法一致。
    bool active_ = false;

#if defined(RM_DEBUG)
    std::unique_ptr<rerun::RecordingStream> stream_;
    std::set<std::string>                   video_codec_paths_;
#endif
};

inline void log(Level level, const std::string &message, const std::string &path)
{
    // 有 Sink 时交给它(控制台 + Rerun);没有 Sink 时只打控制台。
    if (Sink *sink = detail::global_sink())
    {
        sink->log(level, message, path);
        return;
    }
    detail::print_console(level, message);
}

} // namespace tools::debug
