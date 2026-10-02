#include "modules/auto_buff/debug/visualize.hpp"

#include <opencv2/imgproc.hpp>

#include <string>

namespace rm::buff
{
namespace
{

// 关键点显示色:top 绿 / left 黄 / R 红 / right 青 / bottom 品红。
cv::Scalar keypoint_bgr(int index)
{
    switch (index)
    {
    case kpt_top:
        return {0, 255, 0};
    case kpt_left:
        return {0, 255, 255};
    case kpt_r:
        return {0, 0, 255};
    case kpt_right:
        return {255, 255, 0};
    case kpt_bottom:
        return {255, 0, 255};
    default:
        return {255, 255, 255};
    }
}

} // namespace

void draw(cv::Mat &image, const Detector::Result &result)
{
    if (image.empty())
    {
        return;
    }
    const double font_scale = image.rows / 720.0;
    const int    thickness  = 2 + (image.rows / 1080);
    for (const auto &rune : result.runes)
    {
        for (int k = 0; k < KEYPOINT_COUNT; ++k)
        {
            const cv::Point point(cvRound(rune.keypoints[static_cast<std::size_t>(k)].x), cvRound(rune.keypoints[static_cast<std::size_t>(k)].y));
            const int       radius = (k == kpt_r) ? 6 : 4;
            cv::circle(image, point, radius, keypoint_bgr(k), cv::FILLED);
        }
        const cv::Point   anchor(cvRound(rune.keypoints[kpt_top].x), cvRound(rune.keypoints[kpt_top].y));
        const std::string label = std::string(kind_name(rune.kind)) + " " + color_name(rune.color) + " " + cv::format("%.2f", rune.confidence);
        const cv::Scalar  label_color = (rune.color == Color::red) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        cv::putText(image, label, anchor, cv::FONT_HERSHEY_SIMPLEX, font_scale, label_color, thickness);

        // 精修轮廓:装甲板模块绿 / 灯臂青 / 中心 R 黄。
        if (!rune.refinement.armor_module.empty())
        {
            cv::polylines(image, rune.refinement.armor_module, true, {0, 255, 0}, 2);
        }
        if (!rune.refinement.light_arm.empty())
        {
            cv::polylines(image, rune.refinement.light_arm, true, {255, 255, 0}, 2);
        }
        if (!rune.refinement.center_r.empty())
        {
            cv::polylines(image, rune.refinement.center_r, true, {0, 255, 255}, 2);
        }
    }
}

} // namespace rm::buff
