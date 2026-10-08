#include "modules/auto_armor/auto_armor.hpp"

namespace rm::armor
{

AutoAim::AutoAim(const tools::config::Config &config) : detector_(load_detector_config(config)) {}

AutoAim::AutoAim(const DetectorConfig &detector) : detector_(detector) {}

AutoAimResult AutoAim::process(const cv::Mat &bgr, std::optional<Color> enemy_color)
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