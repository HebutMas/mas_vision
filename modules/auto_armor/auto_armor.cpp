#include "modules/auto_armor/auto_armor.hpp"

namespace rm::armor
{

AutoAim::AutoAim(const tools::config::Config &config) : detector_(load_detector_config(config))
{
}

AutoAimResult AutoAim::process(const cv::Mat &bgr, [[maybe_unused]] const double stamp, const std::optional<Color> enemy_color,
                               [[maybe_unused]] const Eigen::Quaterniond &q_imu)
{
    AutoAimResult result;
    if (bgr.empty())
    {
        return result;
    }

    result.detection = detector_.detect(bgr, enemy_color);
    return result;
}

} // namespace rm::armor
