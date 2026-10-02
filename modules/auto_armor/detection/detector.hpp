#pragma once

#include "modules/auto_armor/armor.hpp"
#include "modules/auto_armor/detection/green_light.hpp"
#include "tools/config/config.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace rm::armor
{

struct DetectorConfig
{
    std::string model;                                                            // onnx 路径
    std::string device{"CPU"};                                                    // OpenVINO 设备
    int         input_w{640}, input_h{640};                                       // 输入尺寸
    float       min_confidence{0.5F}, score_threshold{0.7F}, nms_threshold{0.3F}; // 置信度阈值，得分阈值，NMS 阈值
    bool        use_roi{false}; // 是否使用 ROI，若为 true，则在 roi 内进行检测，否则在整张图像上进行检测
    cv::Rect    roi;            // ROI 区域，若 use_roi 为 true，则在 roi 内进行检测，否则在整张图像上进行检测

    GreenLightConfig green_light; // 前哨站 / 基地绿灯滤除(仅在识别到建筑类时触发)

    // 颜色门控:只保留该颜色(敌方色);nullopt 表示不过滤。
    std::optional<Color> enemy_color;
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
    // 禁止拷贝构造与拷贝赋值,单例模式
    Detector(const Detector &)            = delete;
    Detector &operator=(const Detector &) = delete;

    struct Result
    {
        std::vector<Armor2d>    armors;
        std::vector<Lightbar2d> lightbars;
    };

    // enemy_color 覆盖配置里的颜色门控;nullopt 表示沿用配置。
    Result detect(const cv::Mat &bgr, std::optional<Color> enemy_color = std::nullopt);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rm::armor
