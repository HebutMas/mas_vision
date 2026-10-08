#include "modules/auto_buff/auto_buff.hpp"

namespace rm::buff
{

AutoBuff::AutoBuff(const tools::config::Config &config) : detector_(load_detector_config(config)) {}

AutoBuff::AutoBuff(const DetectorConfig &detector) : detector_(detector) {}

AutoBuffResult AutoBuff::process(const cv::Mat &bgr, Color color)
{
    AutoBuffResult result;
    if (bgr.empty())
    {
        return result;
    }

    result.detection = detector_.detect(bgr, color);

    // 选置信度最高的「已激活」符;未激活(inactive)的没有灯臂转动,打不了。
    float best_confidence = -1.0F;
    for (const Rune2d &rune : result.detection.runes)
    {
        if (rune.kind == Kind::inactive)
        {
            continue;
        }
        if (rune.confidence > best_confidence)
        {
            best_confidence = rune.confidence;
            result.target   = rune;
        }
    }

    return result;
}

} // namespace rm::buff
