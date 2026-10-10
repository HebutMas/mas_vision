#include "modules/auto_armor/debug/visualize.hpp"

#include <opencv2/imgproc.hpp>

#include <string>
#include <vector>

namespace rm::armor
{
namespace
{

//   装甲板框 = 黄,装甲板角点 = 绿;
[[nodiscard]] cv::Scalar box_color() noexcept { return {0, 255, 255}; }  // 黄
[[nodiscard]] cv::Scalar corner_color() noexcept { return {0, 255, 0}; } // 绿

} // namespace

void draw(cv::Mat &image, const Detector::Result &result)
{
    if (image.empty())
    {
        return;
    }

    // 线宽/字号随分辨率放大
    const double font_scale = std::max(0.4, image.rows / 1080.0);
    const int    thickness  = 1 + (image.rows / 1080);
    const int    radius     = 3 + (image.rows / 720);

    // 装甲板:黄色框 + 绿色角点 + 小标签
    for (const auto &armor : result.armors)
    {
        std::vector<cv::Point> polygon;
        polygon.reserve(armor.corners.size());
        for (const auto &corner : armor.corners)
        {
            polygon.emplace_back(cvRound(corner.x), cvRound(corner.y));
        }

        cv::polylines(image, polygon, true, box_color(), thickness, cv::LINE_AA);
        for (const auto &corner : armor.corners)
        {
            cv::circle(image, {cvRound(corner.x), cvRound(corner.y)}, radius, corner_color(), cv::FILLED, cv::LINE_AA);
        }

        const std::string label = std::string(kind_name(armor.kind)) + " " + cv::format("%.2f", armor.confidence);
        cv::putText(image, label, {polygon.front().x, polygon.front().y - radius - 2}, cv::FONT_HERSHEY_SIMPLEX, font_scale, box_color(), thickness,
                    cv::LINE_AA);
    }
}

} // namespace rm::armor
