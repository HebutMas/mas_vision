#include "hardware/usbcamera/usbcamera.hpp"

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
} // namespace

UsbCamera::UsbCamera(UsbCameraConfig config) : config_(std::move(config))
{
    open();
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
    // 曝光:0=自动,1=手动;手动模式下再设曝光值与增益。
    capture_.set(cv::CAP_PROP_AUTO_EXPOSURE, config_.auto_exposure == 0 ? EXPOSURE_AUTO : EXPOSURE_MANUAL);
    if (config_.auto_exposure != 0)
    {
        capture_.set(cv::CAP_PROP_EXPOSURE, config_.exposure);
        capture_.set(cv::CAP_PROP_GAIN, config_.gain);
    }
    capture_.set(cv::CAP_PROP_AUTO_WB, config_.auto_wb != 0 ? 1.0 : 0.0);
    capture_.set(cv::CAP_PROP_BUFFERSIZE, BUFFER_SIZE);
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
            // 掉帧或设备异常:退避后重试,避免空转。
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        const TimePoint stamp = tools::time::now();
        frames_.push(UsbFrame{image.clone(), stamp});
    }
}

} // namespace hardware::usbcamera
