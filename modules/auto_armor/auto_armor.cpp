#include "modules/auto_armor/auto_armor.hpp"

#include <cmath>

#include "tools/algorithm/math_tools.hpp"

namespace rm::armor
{

AutoAim::AutoAim(const tools::config::Config &config)
    : detector_(load_detector_config(config)), solver_(load_solver_config(config)), tracker_(track::load_tracker_config(config))
{
}

AutoAimResult AutoAim::process(const cv::Mat &bgr, double stamp, std::optional<Color> enemy_color, const Eigen::Matrix3d &r_gimbal2world)
{
    AutoAimResult result;
    if (bgr.empty())
    {
        return result;
    }

    result.detection = detector_.detect(bgr, enemy_color);
    result.armors.reserve(result.detection.armors.size());
    for (const auto &armor : result.detection.armors)
    {
        if (const auto pose = solver_.solve(armor))
        {
            result.armors.push_back(*pose);
        }
    }

    // 观测旋到世界系再跟踪
    const bool           compensate = !r_gimbal2world.isApprox(Eigen::Matrix3d::Identity());
    const double         gimbal_yaw = compensate ? std::atan2(r_gimbal2world(1, 0), r_gimbal2world(0, 0)) : 0.0;
    std::vector<Armor3d> obs        = result.armors;
    if (compensate)
    {
        for (auto &armor : obs)
        {
            armor.xyz = r_gimbal2world * armor.xyz;
            armor.yaw = tools::limit_rad(armor.yaw + gimbal_yaw);
        }
    }

    auto target = tracker_.track(obs, stamp);
    if (target && compensate)
    {
        target->center = r_gimbal2world.transpose() * target->center;
        target->vel    = r_gimbal2world.transpose() * target->vel;
        target->yaw    = tools::limit_rad(target->yaw - gimbal_yaw);
    }
    result.target = target;
    return result;
}

} // namespace rm::armor