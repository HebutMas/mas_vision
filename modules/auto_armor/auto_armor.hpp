#pragma once

#include <optional>

#include <opencv2/core.hpp>

#include "modules/auto_armor/armor.hpp"
#include "modules/auto_armor/detection/detector.hpp"
#include "tools/config/config.hpp"

namespace rm::armor
{

// 一帧的处理结果:原始检测。
struct AutoAimResult
{
    Detector::Result detection;
};

class AutoAim
{
  public:
    explicit AutoAim(const tools::config::Config &config);
    explicit AutoAim(const DetectorConfig &detector);

    [[nodiscard]] AutoAimResult process(const cv::Mat &bgr, std::optional<Color> enemy_color = std::nullopt);

  private:
    Detector detector_;
};

} // namespace rm::armor