#pragma once

#include <optional>

#include <opencv2/core.hpp>

#include "modules/auto_buff/buff.hpp"
#include "modules/auto_buff/detection/detector.hpp"
#include "tools/config/config.hpp"

namespace rm::buff
{

// 一帧的处理结果:原始检测 + 选中的目标。
struct AutoBuffResult
{
    Detector::Result      detection;
    std::optional<Rune2d> target;
};

class AutoBuff
{
  public:
    explicit AutoBuff(const tools::config::Config &config);
    explicit AutoBuff(const DetectorConfig &detector);

    [[nodiscard]] AutoBuffResult process(const cv::Mat &bgr, Color color = Color::red);

  private:
    Detector detector_;
};

} // namespace rm::buff
