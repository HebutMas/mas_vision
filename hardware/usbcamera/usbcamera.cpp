#include "hardware/usbcamera/usbcamera.hpp"
#include "tools/debug/debug.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <utility>

namespace hardware::usbcamera
{
namespace
{
// V4L2 只保留最新帧,给取图留 1 帧缓冲。
constexpr int BUFFER_SIZE = 1;
// OpenCV V4L2 后端约定:CAP_PROP_AUTO_EXPOSURE 用 0.25=手动、0.75=自动。
constexpr double EXPOSURE_MANUAL = 0.25;
constexpr double EXPOSURE_AUTO   = 0.75;

// "MJPG" -> OpenCV 的四字节 FourCC 整数。
int to_fourcc(const std::string &code)
{
    if (code.size() != 4)
    {
        return 0;
    }
    return cv::VideoWriter::fourcc(code[0], code[1], code[2], code[3]);
}

// FourCC 整数 -> 可读字符串(不可打印字符显示成 '.'),用于日志排查。
std::string from_fourcc(int code)
{
    std::string text(4, '.');
    for (int i = 0; i < 4; ++i)
    {
        const char c                      = static_cast<char>((code >> (8 * i)) & 0xFF);
        text[static_cast<std::size_t>(i)] = (c >= 32 && c < 127) ? c : '.';
    }
    return text;
}
} // namespace

UsbCamera::UsbCamera(UsbCameraConfig config) : config_(std::move(config))
{
    open();
    const int width  = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH));
    const int height = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
    tools::debug::log(tools::debug::Level::info,
                      "usbcamera connected: " + config_.device_path + " " + std::to_string(width) + "x" + std::to_string(height), "usbcamera");
    thread_ = std::thread(&UsbCamera::run, this);
}

UsbCamera::~UsbCamera()
{
    quit_ = true;
    frames_.close();
    if (thread_.joinable())
    {
        thread_.join();
    }
    close();
}

bool UsbCamera::is_open() const noexcept { return capture_.isOpened(); }

void UsbCamera::open()
{
    // 固定 V4L2 后端,避免 OpenCV 回退到 GStreamer 带来的不确定性。
    if (!capture_.open(config_.device_path, cv::CAP_V4L2))
    {
        tools::debug::log(tools::debug::Level::error, "usbcamera open failed: " + config_.device_path, "usbcamera");
        throw std::runtime_error("UsbCamera 无法打开 " + config_.device_path);
    }
    configure();
}

void UsbCamera::configure()
{
    const int fourcc = to_fourcc(config_.fourcc);
    if (fourcc != 0)
    {
        capture_.set(cv::CAP_PROP_FOURCC, fourcc);
    }
    if (config_.width > 0)
    {
        capture_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
    }
    if (config_.height > 0)
    {
        capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
    }
    if (config_.fps > 0)
    {
        capture_.set(cv::CAP_PROP_FPS, config_.fps);
    }
    // 曝光:1=自动,0=手动;手动模式下再设曝光值与增益。
    capture_.set(cv::CAP_PROP_AUTO_EXPOSURE, config_.auto_exposure_enable != 0 ? EXPOSURE_AUTO : EXPOSURE_MANUAL);
    if (config_.auto_exposure_enable == 0)
    {
        capture_.set(cv::CAP_PROP_EXPOSURE, config_.exposure);
        capture_.set(cv::CAP_PROP_GAIN, config_.gain);
    }
    capture_.set(cv::CAP_PROP_AUTO_WB, config_.auto_wb != 0 ? 1.0 : 0.0);
    capture_.set(cv::CAP_PROP_BUFFERSIZE, BUFFER_SIZE);

    // 配置回显打印
    const int         actual_width  = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH));
    const int         actual_height = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
    const int         actual_fourcc = static_cast<int>(capture_.get(cv::CAP_PROP_FOURCC));
    const double      actual_fps    = capture_.get(cv::CAP_PROP_FPS);
    const std::string requested     = config_.fourcc + " " + std::to_string(config_.width) + "x" + std::to_string(config_.height) + "@" +
                                      std::to_string(config_.fps) + (config_.auto_exposure_enable != 0 ? " auto-exposure" : " manual-exposure") +
                                      " exposure=" + std::to_string(config_.exposure) + " gain=" + std::to_string(config_.gain);
    const std::string actual        = from_fourcc(actual_fourcc) + " " + std::to_string(actual_width) + "x" + std::to_string(actual_height) + "@" +
                                      std::to_string(actual_fps) + " exposure=" + std::to_string(capture_.get(cv::CAP_PROP_EXPOSURE)) +
                                      " gain=" + std::to_string(capture_.get(cv::CAP_PROP_GAIN));
    tools::debug::log(tools::debug::Level::info, "usbcamera settings: requested " + requested + " -> actual " + actual, "usbcamera");
    if (config_.width > 0 && actual_width != config_.width)
    {
        tools::debug::log(tools::debug::Level::warn,
                          "usbcamera width not applied: requested " + std::to_string(config_.width) + ", got " + std::to_string(actual_width),
                          "usbcamera");
    }
    if (config_.height > 0 && actual_height != config_.height)
    {
        tools::debug::log(tools::debug::Level::warn,
                          "usbcamera height not applied: requested " + std::to_string(config_.height) + ", got " + std::to_string(actual_height),
                          "usbcamera");
    }
}

void UsbCamera::close() noexcept
{
    if (capture_.isOpened())
    {
        capture_.release();
    }
}

void UsbCamera::run()
{
    cv::Mat image;
    while (!quit_)
    {
        // read 返回后立即记录出流时刻。
        if (!capture_.read(image) || image.empty())
        {
            if (stream_ok_)
            {
                stream_ok_ = false;
                tools::debug::log(tools::debug::Level::warn, "usbcamera read failed: " + config_.device_path, "usbcamera");
            }
            // 掉帧或设备异常:退避后重试,避免空转。
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        if (!stream_ok_)
        {
            stream_ok_ = true;
            tools::debug::log(tools::debug::Level::info, "usbcamera capture recovered", "usbcamera");
        }
        const TimePoint stamp = tools::time::now();
        frames_.push(UsbFrame{image.clone(), stamp});
    }
}

} // namespace hardware::usbcamera
