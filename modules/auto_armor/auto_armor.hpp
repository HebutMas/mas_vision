#pragma once

#include <optional>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "modules/auto_armor/armor.hpp"
#include "modules/auto_armor/detection/detector.hpp"
#include "modules/auto_armor/track/solver.hpp"
#include "modules/auto_armor/track/tracker.hpp"
#include "tools/config/config.hpp"

namespace rm::armor
{

// 一帧自瞄的全部产物。
struct AutoAimResult
{
    Detector::Result                  detection; // 2D 检测(云台系)
    std::vector<Armor3d>              armors;    // PnP 解出的 3D 观测(云台系)
    std::optional<track::TargetState> target;    // 整车跟踪结果(云台系;空 = 丢失)
};

class AutoAim
{
  public:
    explicit AutoAim(const tools::config::Config &config);

    [[nodiscard]] AutoAimResult process(const cv::Mat &bgr, double stamp, std::optional<Color> enemy_color = std::nullopt,
                                        const Eigen::Matrix3d &r_gimbal2world = Eigen::Matrix3d::Identity());

  private:
    Detector       detector_;
    Solver         solver_;
    track::Tracker tracker_;
};

} // namespace rm::armor