#pragma once

#include "modules/auto_buff/buff.hpp"
#include "tools/config/config.hpp"

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace rm::buff
{

struct DetectorConfig
{
    std::string model;                               // onnx 路径
    std::string device{"CPU"};                     // OpenVINO 设备
    float       confidence_threshold{0.8F};          // 类别置信度阈值
    float       keypoint_confidence_threshold{0.8F}; // 单关键点置信度阈值
    float       nms_distance_threshold{30.0F};       // 中心距 NMS 阈值(像素)
    int         min_valid_keypoints{3};              // 有效关键点少于此数则丢弃
};

// 从配置读取检测器参数。
[[nodiscard]] DetectorConfig load_detector_config(const tools::config::Config &config);

class Detector
{
  public:
    explicit Detector(const DetectorConfig &config);
    ~Detector();

    Detector(Detector &&) noexcept;
    Detector &operator=(Detector &&) noexcept;
    Detector(const Detector &)            = delete;
    Detector &operator=(const Detector &) = delete;

    struct Result
    {
        std::vector<Rune2d> runes;
    };

    // color 由下发的 VisionMode 决定,写入每个识别结果。
    [[nodiscard]] Result detect(const cv::Mat &bgr, Color color = Color::red);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rm::buff
