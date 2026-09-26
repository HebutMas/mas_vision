#include "hardware/hikcamera/hikcamera.hpp"

#include "MvCameraControl.h"

#include <opencv2/imgproc.hpp>

#include <cstddef>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace hardware::hikcamera
{
namespace
{
// SDK 内部缓存节点数(必须在开始取流前设置)。
// 配合 LatestImagesOnly 策略:只保留最新帧,给转换期间留 1 帧余量即可。
constexpr unsigned int NODE_NUM = 2;
// 单次取图超时(毫秒)。
constexpr unsigned int GRAB_TIMEOUT_MS = 200;

std::string error_text(unsigned int code)
{
  char buffer[16];
  std::snprintf(buffer, sizeof(buffer), "0x%08X", code);
  return buffer;
}

void check(unsigned int code, const char * what)
{
  if (code != MV_OK)
  {
    throw std::runtime_error(std::string("HikCamera ") + what + " 失败:" + error_text(code));
  }
}

// USB 2.0 直接拒绝启动。
void check_usb3(const MV_CC_DEVICE_INFO & device)
{
  if (device.nTLayerType != MV_USB_DEVICE)
  {
    return;
  }
  const unsigned int usb = device.SpecialInfo.stUsb3VInfo.nbcdUSB;
  if (usb < 0x0300)
  {
    throw std::runtime_error(
        "The HikCamera camera is connected via USB 2.0(" + error_text(usb) +
        "), Bandwidth is limited. Please connect to the USB 3.0 port instead.");
  }
}

bool is_mono(MvGvspPixelType type)
{
  static const std::unordered_set<MvGvspPixelType> mono_set = {
      PixelType_Gvsp_Mono1p,        PixelType_Gvsp_Mono2p,       PixelType_Gvsp_Mono4p,
      PixelType_Gvsp_Mono8,         PixelType_Gvsp_Mono8_Signed, PixelType_Gvsp_Mono10,
      PixelType_Gvsp_Mono10_Packed, PixelType_Gvsp_Mono12,       PixelType_Gvsp_Mono12_Packed,
      PixelType_Gvsp_Mono14,        PixelType_Gvsp_Mono16};
  return mono_set.count(type) != 0;
}

// Bayer8 -> BGR 的 OpenCV 转换码;非 Bayer8 返回 -1。
int bayer_code(MvGvspPixelType type)
{
  switch (type)
  {
  case PixelType_Gvsp_BayerGR8:
    return cv::COLOR_BayerGR2BGR;
  case PixelType_Gvsp_BayerRG8:
    return cv::COLOR_BayerRG2BGR;
  case PixelType_Gvsp_BayerGB8:
    return cv::COLOR_BayerGB2BGR;
  case PixelType_Gvsp_BayerBG8:
    return cv::COLOR_BayerBG2BGR;
  default:
    return -1;
  }
}

// 把一帧 SDK 原始数据转成 OpenCV 图像。
cv::Mat to_cv(void * handle, const MV_FRAME_OUT & raw)
{
  const auto & info = raw.stFrameInfo;
  const int width = static_cast<int>(info.nWidth);
  const int height = static_cast<int>(info.nHeight);

  // Mono8 直接包装成 GRAY。
  if (info.enPixelType == PixelType_Gvsp_Mono8)
  {
    return cv::Mat(height, width, CV_8UC1, raw.pBufAddr).clone();
  }

  // 彩色。
  const int code = bayer_code(info.enPixelType);
  if (code >= 0)
  {
    const cv::Mat bayer(height, width, CV_8UC1, raw.pBufAddr);
    cv::Mat bgr;
    cv::cvtColor(bayer, bgr, code);
    return bgr;
  }

  // 其他格式(YUV / RGB / 高位深 mono 等):SDK 转 8 位。
  const bool mono = is_mono(info.enPixelType);
  std::vector<std::uint8_t> buffer(static_cast<std::size_t>(width) * height * (mono ? 1 : 3));
  MV_CC_PIXEL_CONVERT_PARAM param{}; // NOLINT(bugprone-invalid-enum-default-initialization)
  param.nWidth = info.nWidth;
  param.nHeight = info.nHeight;
  param.pSrcData = raw.pBufAddr;
  param.nSrcDataLen = info.nFrameLen;
  param.enSrcPixelType = info.enPixelType;
  param.enDstPixelType = mono ? PixelType_Gvsp_Mono8 : PixelType_Gvsp_BGR8_Packed;
  param.pDstBuffer = buffer.data();
  param.nDstBufferSize = static_cast<unsigned int>(buffer.size());
  if (MV_CC_ConvertPixelType(handle, &param) != MV_OK)
  {
    return {};
  }
  return cv::Mat(height, width, mono ? CV_8UC1 : CV_8UC3, buffer.data()).clone();
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

bool HikCamera::is_open() const noexcept
{
  return handle_ != nullptr;
}

void HikCamera::open()
{
  try
  {
    MV_CC_DEVICE_INFO_LIST devices{};
    check(MV_CC_EnumDevices(MV_USB_DEVICE, &devices), "MV_CC_EnumDevices");
    if (devices.nDeviceNum == 0)
    {
      throw std::runtime_error("HikCamera 未找到 USB 相机");
    }

    const MV_CC_DEVICE_INFO * device = nullptr;
    if (config_.serial.empty())
    {
      device = devices.pDeviceInfo[0];
    }
    else
    {
      for (unsigned int i = 0; i < devices.nDeviceNum && device == nullptr; ++i)
      {
        const MV_CC_DEVICE_INFO * info = devices.pDeviceInfo[i];
        const bool matched = info != nullptr && info->nTLayerType == MV_USB_DEVICE &&
                             config_.serial == reinterpret_cast<const char *>(
                                                   info->SpecialInfo.stUsb3VInfo.chSerialNumber);
        if (matched)
        {
          device = info;
        }
      }
      if (device == nullptr)
      {
        throw std::runtime_error("HikCamera 未找到序列号为 " + config_.serial + " 的相机");
      }
    }

    check(MV_CC_CreateHandle(&handle_, device), "MV_CC_CreateHandle");
    check(MV_CC_OpenDevice(handle_), "MV_CC_OpenDevice");
    check_usb3(*device);
    configure();
    check(MV_CC_StartGrabbing(handle_), "MV_CC_StartGrabbing");
  }
  catch (...)
  {
    close();
    throw;
  }
}

void HikCamera::configure()
{
  // 连续采集。
  MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
  // 曝光与增益与白平衡配置决定。
  MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  MV_CC_SetFloatValue(handle_, "ExposureTime", config_.exposure_us);
  MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
  MV_CC_SetFloatValue(handle_, "Gain", config_.gain_db);
  MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto",
                     config_.auto_white_balance ? MV_BALANCEWHITE_AUTO_CONTINUOUS
                                                : MV_BALANCEWHITE_AUTO_OFF);
  MV_CC_SetFrameRate(handle_, static_cast<float>(config_.frame_rate));
  MV_CC_SetImageNodeNum(handle_, NODE_NUM);
  // 只取最新帧。
  MV_CC_SetGrabStrategy(handle_, MV_GrabStrategy_LatestImagesOnly);
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
    MV_FRAME_OUT raw{}; // NOLINT(bugprone-invalid-enum-default-initialization)
    const unsigned int code = MV_CC_GetImageBuffer(handle_, &raw, GRAB_TIMEOUT_MS);
    if (quit_)
    {
      break;
    }
    if (code != MV_OK)
    {
      // 超时或设备异常:退避后重试,避免空转。
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    // 取出原始数据后立即记录出流时刻。
    const TimePoint stamp = tools::time::now();
    cv::Mat frame = to_cv(handle_, raw);
    MV_CC_FreeImageBuffer(handle_, &raw);

    if (!frame.empty())
    {
      frames_.push(HikFrame{std::move(frame), stamp});
    }
  }
}

} // namespace hardware::hikcamera
