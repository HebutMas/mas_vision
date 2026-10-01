#include "tools/debug/record/record.hpp"

#include "tools/debug/debug.hpp"
#include "tools/debug/video/video_encoder.hpp"
#include "tools/time/time.hpp"

#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <system_error>
#include <utility>

namespace tools::record
{

namespace
{

namespace fs = std::filesystem;

// 可读的本地时间文件名,如 autoaim_2026-10-01_13-45-00。
std::string make_name()
{
    const std::time_t now = std::time(nullptr);
    std::tm           tm{};
    localtime_r(&now, &tm);

    std::ostringstream out;
    out << "autoaim_" << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    return out.str();
}

} // namespace

struct Recorder::Impl
{
    RecordConfig config;

    bool        active         = false;
    bool        header_written = false;
    bool        opened_set     = false;
    std::string note; // start() 失败原因 / 自动停止原因,供 status() 显示
    std::string name; // 不带扩展名的文件名
    fs::path    directory;

    std::ofstream   video;
    std::ofstream   csv;
    std::uintmax_t  bytes = 0;
    time::TimePoint opened;

    std::unique_ptr<video::VideoEncoder> encoder; // 首帧尺寸确定后才构造

    [[nodiscard]] std::string video_path() const { return (directory / (name + ".h264")).string(); }
    [[nodiscard]] std::string csv_path() const { return (directory / (name + ".csv")).string(); }

    // 统一的启动失败出口:记录原因、打日志、返回 false。
    bool fail(std::string reason)
    {
        note = "record start failed: " + std::move(reason);
        debug::log(debug::Level::error, note, "record");
        return false;
    }

    // 依次尝试配置目录,返回第一个可用的;都不可用则返回空路径。
    [[nodiscard]] fs::path select_directory() const
    {
        for (const std::string &candidate : config.directories)
        {
            std::error_code ec;
            const fs::path  path(candidate);
            if (!fs::exists(path, ec))
            {
                fs::create_directories(path, ec);
            }
            if (!ec && fs::is_directory(path))
            {
                return path;
            }
        }
        return {};
    }

    // 目录内已有录像(视频 + CSV)的总体积。
    [[nodiscard]] std::uintmax_t recorded_size() const
    {
        std::uintmax_t  total = 0;
        std::error_code ec;
        for (const fs::directory_entry &entry : fs::directory_iterator(directory, ec))
        {
            const fs::path &path = entry.path();
            if (entry.is_regular_file(ec) && (path.extension() == ".h264" || path.extension() == ".csv"))
            {
                total += entry.file_size(ec);
            }
        }
        return total;
    }

    bool start()
    {
        note.clear();
        if (active)
        {
            stop(true);
        }

        directory = select_directory();
        if (directory.empty())
        {
            return fail("no usable record directory");
        }

        std::error_code      ec;
        const fs::space_info space = fs::space(directory, ec);
        if (!ec && space.available < config.min_free_space)
        {
            return fail("not enough free space in " + directory.string());
        }
        if (recorded_size() >= config.max_total_size)
        {
            return fail("record directory size limit reached: " + directory.string());
        }

        // 同一秒内重复开始会重名,加序号避免覆盖。
        const std::string base = make_name();
        name                   = base;
        for (std::uint32_t index = 1; fs::exists(directory / (name + ".h264"), ec); ++index)
        {
            name = base + "_" + std::to_string(index);
        }

        video.open(video_path(), std::ios::binary | std::ios::trunc);
        csv.open(csv_path(), std::ios::trunc);
        if (!video || !csv)
        {
            video.close();
            csv.close();
            return fail("cannot create record files in " + directory.string());
        }
        csv << std::setprecision(10);

        // 提前固定时间基准,保证之后所有帧时间戳相对同一原点、非负。
        static_cast<void>(time::base());

        bytes          = 0;
        header_written = false;
        opened_set     = false;
        active         = true;
        debug::log(debug::Level::info, "record started: " + video_path(), "record");
        return true;
    }

    // 首帧到达时按帧尺寸构造编码器。
    bool start_encoder(const cv::Mat &bgr)
    {
        video::VideoEncoderConfig encoder_config;
        encoder_config.width   = bgr.cols;
        encoder_config.height  = bgr.rows;
        encoder_config.fps     = config.fps;
        encoder_config.bitrate = config.bitrate;
        encoder_config.gop     = config.gop;

        try
        {
            encoder = std::make_unique<video::VideoEncoder>(encoder_config);
        }
        catch (const std::exception &error)
        {
            note = std::string("record start failed: cannot create video encoder: ") + error.what();
            debug::log(debug::Level::error, note, "record");
            stop(false);
            return false;
        }

        debug::log(debug::Level::info,
                   "record video: " + video_path() + " " + std::to_string(bgr.cols) + "x" + std::to_string(bgr.rows) + " (" +
                       (encoder->hardware() ? "VAAPI" : "CPU") + ")",
                   "record");
        return true;
    }

    void stop(bool save)
    {
        if (!active)
        {
            return;
        }
        active = false;

        if (encoder != nullptr)
        {
            try
            {
                for (const video::EncodedFrame &frame : encoder->flush())
                {
                    video.write(reinterpret_cast<const char *>(frame.data.data()), static_cast<std::streamsize>(frame.data.size()));
                    bytes += frame.data.size();
                }
            }
            catch (const std::exception &error)
            {
                debug::log(debug::Level::warn, std::string("record failed: flush: ") + error.what(), "record");
            }
            encoder.reset();
        }

        video.close();
        csv.close();

        if (save)
        {
            debug::log(debug::Level::info, "record stopped: " + video_path() + " saved (" + std::to_string(bytes) + " bytes)", "record");
        }
        else
        {
            std::error_code ec;
            fs::remove(video_path(), ec);
            fs::remove(csv_path(), ec);
            debug::log(debug::Level::info, "record stopped: " + video_path() + " discarded", "record");
        }
    }

    void push(const cv::Mat &bgr, time::TimePoint timestamp, const std::vector<double> &values)
    {
        if (!active)
        {
            return;
        }

        if (!opened_set)
        {
            opened     = time::now();
            opened_set = true;
        }
        else if (time::now() - opened >= config.max_duration)
        {
            note = "record duration limit reached: " + video_path();
            stop(true);
            return;
        }

        if (bgr.empty() || (encoder == nullptr && !start_encoder(bgr)))
        {
            return;
        }

        if (!header_written)
        {
            csv << "t_ns";
            for (const std::string &column : config.columns)
            {
                csv << ',' << column;
            }
            csv << '\n';
            header_written = true;
        }
        csv << time::since_base(timestamp).count();
        for (const double value : values)
        {
            csv << ',' << value;
        }
        csv << '\n';

        try
        {
            for (const video::EncodedFrame &frame : encoder->encode(bgr, time::since_base(timestamp)))
            {
                video.write(reinterpret_cast<const char *>(frame.data.data()), static_cast<std::streamsize>(frame.data.size()));
                bytes += frame.data.size();
            }
        }
        catch (const std::exception &error)
        {
            note = std::string("record failed: encode: ") + error.what();
            debug::log(debug::Level::error, note, "record");
            stop(true);
        }
    }
};

Recorder::Recorder(RecordConfig config) : impl_(std::make_unique<Impl>()) { impl_->config = std::move(config); }

Recorder::~Recorder()
{
    if (impl_->active)
    {
        impl_->stop(true);
    }
}

bool Recorder::start() { return impl_->start(); }

void Recorder::stop(bool save) { impl_->stop(save); }

bool Recorder::recording() const { return impl_->active; }

std::optional<std::string> Recorder::filename() const
{
    if (!impl_->active)
    {
        return std::nullopt;
    }
    return impl_->video_path();
}

std::string Recorder::status() const
{
    if (impl_->active)
    {
        return "recording: " + impl_->video_path();
    }
    return impl_->note.empty() ? "not recording" : impl_->note;
}

void Recorder::push(const cv::Mat &bgr, tools::time::TimePoint timestamp, const std::vector<double> &values) { impl_->push(bgr, timestamp, values); }

} // namespace tools::record
