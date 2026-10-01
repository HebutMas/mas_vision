#include "hardware/hikcamera/hikcamera.hpp"

#include "MvCameraControl.h"
#include "tools/debug/debug.hpp"

#include <opencv2/imgproc.hpp>

#include <cstdio>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

namespace hardware::hikcamera
{
namespace
{
// SDK 内部缓存节点数(必须在开始取流前设置)。
constexpr unsigned int NODE_NUM = 3;
// 单次取图超时(毫秒)。
constexpr unsigned int GRAB_TIMEOUT_MS = 200;

std::string error_text(unsigned int code)
{
    char buffer[16];
    std::snprintf(buffer, sizeof(buffer), "0x%08X", code);
    return buffer;
}

void check(unsigned int code, const char *what)
{
    if (code != MV_OK)
    {
        throw std::runtime_error(std::string("HikCamera ") + what + " failed:" + error_text(code));
    }
}

// USB 2.0 直接拒绝启动。
void check_usb3(const MV_CC_DEVICE_INFO &device)
{
    if (device.nTLayerType != MV_USB_DEVICE)
    {
        return;
    }
    const unsigned int usb = device.SpecialInfo.stUsb3VInfo.nbcdUSB;
    if (usb < 0x0300)
    {
        throw std::runtime_error("The HikCamera camera is connected via USB 2.0(" + error_text(usb) +
                                 "), Bandwidth is limited. Please connect to the USB 3.0 port instead.");
    }
}

bool is_mono(MvGvspPixelType type)
{
    static const std::unordered_set<MvGvspPixelType> mono_set = {
        PixelType_Gvsp_Mono1p,        PixelType_Gvsp_Mono2p, PixelType_Gvsp_Mono4p,        PixelType_Gvsp_Mono8,
        PixelType_Gvsp_Mono8_Signed,  PixelType_Gvsp_Mono10, PixelType_Gvsp_Mono10_Packed, PixelType_Gvsp_Mono12,
        PixelType_Gvsp_Mono12_Packed, PixelType_Gvsp_Mono14, PixelType_Gvsp_Mono16};
    return mono_set.count(type) != 0;
}

// Bayer8 -> BGR 的 OpenCV 转换码;非 Bayer8 返回 -1。
// SDK 的 Bayer 命名与 OpenCV 的转换码是对调的(RG↔BG、GR↔GB):
// OpenCV 的 COLOR_BayerBG2BGR 对应 RGGB 阵列、COLOR_BayerRG2BGR 对应 BGGR 阵列。
int bayer_code(MvGvspPixelType type, DemosaicQuality quality)
{
    const bool ea = quality == DemosaicQuality::edge_aware;
    switch (type)
    {
    case PixelType_Gvsp_BayerGR8:
        return ea ? cv::COLOR_BayerGB2BGR_EA : cv::COLOR_BayerGB2BGR;
    case PixelType_Gvsp_BayerRG8:
        return ea ? cv::COLOR_BayerBG2BGR_EA : cv::COLOR_BayerBG2BGR;
    case PixelType_Gvsp_BayerGB8:
        return ea ? cv::COLOR_BayerGR2BGR_EA : cv::COLOR_BayerGR2BGR;
    case PixelType_Gvsp_BayerBG8:
        return ea ? cv::COLOR_BayerRG2BGR_EA : cv::COLOR_BayerRG2BGR;
    default:
        return -1;
    }
}

// 把一帧 SDK 原始数据转成 OpenCV 图像。每次返回独立拥有数据的 Mat。
cv::Mat to_cv(void *handle, const MV_FRAME_OUT &raw, DemosaicQuality quality)
{
    const auto &info   = raw.stFrameInfo;
    const int   width  = static_cast<int>(info.nWidth);
    const int   height = static_cast<int>(info.nHeight);

    // Mono8 直接包装成 GRAY。
    if (info.enPixelType == PixelType_Gvsp_Mono8)
    {
        return cv::Mat(height, width, CV_8UC1, raw.pBufAddr).clone();
    }

    // Bayer8:OpenCV 转换
    const int code = bayer_code(info.enPixelType, quality);
    if (code >= 0)
    {
        const cv::Mat bayer(height, width, CV_8UC1, raw.pBufAddr);
        cv::Mat       bgr;
        cv::cvtColor(bayer, bgr, code);
        return bgr;
    }

    // 其他格式(YUV / RGB / 高位深 mono 等):SDK 转 8 位,直接写入目标 Mat。
    const bool                mono = is_mono(info.enPixelType);
    cv::Mat                   dst(height, width, mono ? CV_8UC1 : CV_8UC3);
    MV_CC_PIXEL_CONVERT_PARAM param{}; // NOLINT(bugprone-invalid-enum-default-initialization)
    param.nWidth         = info.nWidth;
    param.nHeight        = info.nHeight;
    param.pSrcData       = raw.pBufAddr;
    param.nSrcDataLen    = info.nFrameLen;
    param.enSrcPixelType = info.enPixelType;
    param.enDstPixelType = mono ? PixelType_Gvsp_Mono8 : PixelType_Gvsp_BGR8_Packed;
    param.pDstBuffer     = dst.data;
    param.nDstBufferSize = static_cast<unsigned int>(dst.total() * dst.elemSize());
    if (MV_CC_ConvertPixelType(handle, &param) != MV_OK)
    {
        return {};
    }
    return dst;
}
} // namespace

HikCamera::HikCamera(HikCameraConfig config) : config_(std::move(config))
{
    open();
    thread_ = std::thread(&HikCamera::run, this);
}

HikCamera::~HikCamera()
{
    quit_ = true;
    frames_.close();
    if (thread_.joinable())
    {
        thread_.join();
    }
    close();
}

bool HikCamera::is_open() const noexcept { return handle_ != nullptr; }

void HikCamera::open()
{
    try
    {
        MV_CC_DEVICE_INFO_LIST devices{};
        check(MV_CC_EnumDevices(MV_USB_DEVICE, &devices), "MV_CC_EnumDevices");
        if (devices.nDeviceNum == 0)
        {
            throw std::runtime_error("HikCamera no USB camera found");
        }

        const MV_CC_DEVICE_INFO *device = nullptr;
        if (config_.serial.empty())
        {
            device = devices.pDeviceInfo[0];
        }
        else
        {
            for (unsigned int i = 0; i < devices.nDeviceNum && device == nullptr; ++i)
            {
                const MV_CC_DEVICE_INFO *info    = devices.pDeviceInfo[i];
                const bool               matched = info != nullptr && info->nTLayerType == MV_USB_DEVICE &&
                                                   config_.serial == reinterpret_cast<const char *>(info->SpecialInfo.stUsb3VInfo.chSerialNumber);
                if (matched)
                {
                    device = info;
                }
            }
            if (device == nullptr)
            {
                throw std::runtime_error("HikCamera no USB camera found with serial number: " + config_.serial);
            }
        }

        check(MV_CC_CreateHandle(&handle_, device), "MV_CC_CreateHandle");
        check(MV_CC_OpenDevice(handle_), "MV_CC_OpenDevice");
        check_usb3(*device);
        configure();
        check(MV_CC_StartGrabbing(handle_), "MV_CC_StartGrabbing");
        tools::debug::log(tools::debug::Level::info,
                          "hikcamera connected: serial=" +
                              std::string(reinterpret_cast<const char *>(device->SpecialInfo.stUsb3VInfo.chSerialNumber)),
                          "hikcamera");
    }
    catch (...)
    {
        close();
        throw;
    }
}

void HikCamera::configure()
{
    // 每一项都检查返回值:静默失败的设置会让相机停在未知状态
    // 连续采集
    check(MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF), "set trigger mode");
    // 曝光与增益与白平衡配置。
    check(MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF), "disable auto exposure");
    check(MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(config_.exposure_us)), "set exposure time");
    check(MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF), "disable auto gain");
    check(MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(config_.gain_db)), "set gain");
    // 白平衡固定关闭。
    check(MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_OFF), "disable auto white balance");
    // 帧率:framerate>0 时启用并设为目标值,否则不限制、跑相机最大帧率。
    if (config_.framerate > 0.0)
    {
        check(MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true), "enable frame rate");
        check(MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", static_cast<float>(config_.framerate)), "set frame rate");
    }
    else
    {
        check(MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", false), "disable frame rate limit");
    }

    // 相机侧裁切;不开 ROI 时显式回到全幅,避免相机里残留的 ROI 让画面异常。
    MVCC_INTVALUE_EX size{};
    if (config_.roi.enable)
    {
        check(MV_CC_SetIntValueEx(handle_, "OffsetX", config_.roi.x), "set roi offset x");
        check(MV_CC_SetIntValueEx(handle_, "OffsetY", config_.roi.y), "set roi offset y");
        if (config_.roi.width > 0)
        {
            check(MV_CC_SetIntValueEx(handle_, "Width", config_.roi.width), "set roi width");
        }
        if (config_.roi.height > 0)
        {
            check(MV_CC_SetIntValueEx(handle_, "Height", config_.roi.height), "set roi height");
        }
    }
    else
    {
        check(MV_CC_SetIntValueEx(handle_, "OffsetX", 0), "reset roi offset x");
        check(MV_CC_SetIntValueEx(handle_, "OffsetY", 0), "reset roi offset y");
        check(MV_CC_GetIntValueEx(handle_, "WidthMax", &size), "get width max");
        check(MV_CC_SetIntValueEx(handle_, "Width", size.nCurValue), "reset roi width");
        check(MV_CC_GetIntValueEx(handle_, "HeightMax", &size), "get height max");
        check(MV_CC_SetIntValueEx(handle_, "Height", size.nCurValue), "reset roi height");
    }

    check(MV_CC_SetImageNodeNum(handle_, NODE_NUM), "set image node num");
    // 只取最新帧。
    check(MV_CC_SetGrabStrategy(handle_, MV_GrabStrategy_LatestImages), "set grab strategy");
    // check(MV_CC_SetOutputQueueSize(handle_, 1), "set output queue size");
}

void HikCamera::close() noexcept
{
    if (handle_ == nullptr)
    {
        return;
    }
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
}

void HikCamera::run()
{
    while (!quit_)
    {
        MV_FRAME_OUT       raw{}; // NOLINT(bugprone-invalid-enum-default-initialization)
        const unsigned int code = MV_CC_GetImageBuffer(handle_, &raw, GRAB_TIMEOUT_MS);
        if (quit_)
        {
            break;
        }
        if (code != MV_OK)
        {
            if (capture_ok_)
            {
                capture_ok_ = false;
                tools::debug::log(tools::debug::Level::warn, "hikcamera grab failed: " + error_text(code), "hikcamera");
            }
            // 超时或设备异常:退避后重试,避免空转。
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // 取出原始数据后立即记录出流时刻。
        const TimePoint stamp      = tools::time::now();
        const int       width      = raw.stFrameInfo.nWidth;
        const int       height     = raw.stFrameInfo.nHeight;
        const auto      pixel_type = static_cast<unsigned int>(raw.stFrameInfo.enPixelType);
        cv::Mat         frame      = to_cv(handle_, raw, config_.demosaic);
        MV_CC_FreeImageBuffer(handle_, &raw);

        if (frame.empty())
        {
            if (capture_ok_)
            {
                capture_ok_ = false;
                tools::debug::log(tools::debug::Level::warn, "hikcamera convert failed: pixel_type=" + error_text(pixel_type), "hikcamera");
            }
            continue;
        }
        if (!capture_ok_)
        {
            capture_ok_ = true;
            tools::debug::log(tools::debug::Level::info, "hikcamera capture recovered", "hikcamera");
        }
        if (!first_frame_logged_)
        {
            first_frame_logged_ = true;
            tools::debug::log(tools::debug::Level::info,
                              "hikcamera first frame: " + std::to_string(width) + "x" + std::to_string(height) +
                                  " pixel_type=" + error_text(pixel_type),
                              "hikcamera");
        }

        frames_.push(HikFrame{std::move(frame), stamp});
    }
}

} // namespace hardware::hikcamera
