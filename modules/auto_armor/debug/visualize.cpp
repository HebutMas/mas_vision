#include "modules/auto_armor/debug/visualize.hpp"

#include <opencv2/imgproc.hpp>

#include <string>
#include <vector>

namespace rm::armor
{
namespace
{

// 识别颜色 -> BGR 显示色(红/蓝/灰/紫)。
cv::Scalar armor_color_bgr(Color color)
{
    switch (color)
    {
    case Color::red:
        return {0, 0, 255};
    case Color::blue:
        return {255, 0, 0};
    case Color::gray:
        return {128, 128, 128};
    case Color::purple:
        return {255, 0, 255};
    }
    return {255, 255, 255};
}

} // namespace

void draw(cv::Mat &image, const Detector::Result &result)
{
    if (image.empty())
    {
        return;
    }
    for (const auto &armor : result.armors)
    {
        std::vector<cv::Point> polygon;
        polygon.reserve(armor.corners.size());
        for (const auto &corner : armor.corners)
        {
            polygon.emplace_back(cvRound(corner.x), cvRound(corner.y));
        }
        const cv::Scalar  color = armor_color_bgr(armor.color);
        const std::string label = std::string(kind_name(armor.kind)) + " " + color_name(armor.color) + " " + cv::format("%.2f", armor.confidence);
        // 字体随分辨率放大
        const double font_scale = image.rows / 720.0;
        const int    thickness  = 2 + (image.rows / 1080);
        cv::putText(image, label, polygon.front(), cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
    }
}

} // namespace rm::armor
