#include "modules/auto_armor/detection/green_light.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace rm::armor
{

namespace
{

constexpr float  EXPAND_SCALE = 4.0F; // 搜索区域向下扩展倍数
constexpr double GREEN_MARGIN = 30.0; // G 需高出 R/B 的余量
constexpr double PI           = 3.14159265358979323846;

} // namespace

GreenLightResult find_green_light(const cv::Mat &bgr, const std::vector<Armor2d> &armors, const GreenLightConfig &config)
{
    GreenLightResult result;
    if (armors.empty() || bgr.empty())
    {
        return result;
    }

    // 1. 包围所有装甲板并带拓展的搜索区域。
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    float max_w = 0.0F;
    float max_h = 0.0F;
    for (const auto &armor : armors)
    {
        const cv::Rect rect = cv::boundingRect(std::vector<cv::Point2f>(armor.corners.begin(), armor.corners.end()));
        max_w               = std::max(max_w, static_cast<float>(rect.width));
        max_h               = std::max(max_h, static_cast<float>(rect.height));
        for (const auto &point : armor.corners)
        {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
        }
    }

    const float left   = min_x - max_w - 20.0F;
    const float right  = max_x + max_w + 20.0F;
    const float top    = min_y - max_h - 20.0F;
    const float bottom = max_y + (max_h * EXPAND_SCALE);

    const cv::Rect roi = cv::Rect(static_cast<int>(std::floor(left)), static_cast<int>(std::floor(top)),
                                  static_cast<int>(std::ceil(right) - std::floor(left)), static_cast<int>(std::ceil(bottom) - std::floor(top))) &
                         cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (roi.width <= 0 || roi.height <= 0)
    {
        return result;
    }
    result.roi = roi;

    // 2. 绿色掩码:G 够亮,且 G 明显高于 R/B。
    const cv::Mat source = bgr(roi);
    cv::Mat       b;
    cv::Mat       g;
    cv::Mat       r;
    cv::extractChannel(source, b, 0);
    cv::extractChannel(source, g, 1);
    cv::extractChannel(source, r, 2);

    cv::Mat mask_g_min;
    cv::Mat mask_g_gt_r;
    cv::Mat mask_g_gt_b;
    cv::threshold(g, mask_g_min, config.green_threshold, 255.0, cv::THRESH_BINARY);
    cv::subtract(g, r, mask_g_gt_r);
    cv::subtract(g, b, mask_g_gt_b);
    cv::threshold(mask_g_gt_r, mask_g_gt_r, GREEN_MARGIN, 255.0, cv::THRESH_BINARY);
    cv::threshold(mask_g_gt_b, mask_g_gt_b, GREEN_MARGIN, 255.0, cv::THRESH_BINARY);

    cv::Mat mask;
    cv::bitwise_and(mask_g_min, mask_g_gt_r, mask);
    cv::bitwise_and(mask, mask_g_gt_b, mask);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5}));

    // 3. 按面积 / 圆度 / 长宽比挑选最像绿灯的轮廓。
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_area = 0.0;
    for (const auto &contour : contours)
    {
        const double area = cv::contourArea(contour);
        if (area < config.min_area)
        {
            continue;
        }
        const double perimeter = cv::arcLength(contour, true);
        if (!(perimeter > 0.0))
        {
            continue;
        }
        const double circularity = 4.0 * PI * area / (perimeter * perimeter);
        if (circularity < config.min_circularity)
        {
            continue;
        }
        const cv::Rect rect = cv::boundingRect(contour);
        if (rect.width <= 0 || rect.height <= 0)
        {
            continue;
        }
        const double aspect = 1.0 * std::max(rect.width, rect.height) / std::min(rect.width, rect.height);
        if (aspect > config.max_aspect_ratio)
        {
            continue;
        }
        if (area > best_area)
        {
            best_area          = area;
            result.green_light = rect + roi.tl();
        }
    }
    return result;
}

void filter_buildings_above_green_light(std::vector<Armor2d> &armors, const cv::Rect &green_light)
{
    const auto is_higher_building = [&green_light](const Armor2d &armor) {
        const bool building = armor.kind == Kind::outpost || armor.kind == Kind::base;
        return building && armor.corners[3].y < static_cast<float>(green_light.y);
    };
    armors.erase(std::remove_if(armors.begin(), armors.end(), is_higher_building), armors.end());
}

} // namespace rm::armor
