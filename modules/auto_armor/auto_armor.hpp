#pragma once

#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "modules/auto_armor/armor.hpp"
#include "modules/auto_armor/detection/detector.hpp"
#include "tools/config/config.hpp"

namespace rm::armor
{

// 一帧自瞄的全部产物。
struct AutoAimResult
{
    Detector::Result detection; // 2D 检测
};

class AutoAim
{
  public:
    explicit AutoAim(const tools::config::Config &config);

    // q_imu:本帧图像曝光时刻的**原始 IMU 姿态**(下位机透传,未做安装补偿)。
    // 目前跟踪层已移除、暂未使用;重新规划 track 时由跟踪层接手。
    [[nodiscard]] AutoAimResult process(const cv::Mat &bgr, double stamp, std::optional<Color> enemy_color,
                                        const Eigen::Quaterniond &q_imu);

  private:
    Detector detector_;
};

} // namespace rm::armor
