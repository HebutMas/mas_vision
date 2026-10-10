#include "modules/auto_buff/detection/refiner.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rm::buff
{
namespace
{

constexpr double PI = 3.14159265358979323846;
// 轮廓点数过少(二值化噪声)直接跳过。
constexpr int MIN_CONTOUR_POINTS = 5;
// 已激活灯臂(山形)多边形近似的拐点数区间。
constexpr int MIN_BIG_APPROX_POINTS   = 8;
constexpr int MAX_BIG_APPROX_POINTS   = 16;
constexpr int MIN_SMALL_APPROX_POINTS = 12;
constexpr int MAX_SMALL_APPROX_POINTS = 16;

bool point_in_contour(const std::vector<cv::Point> &contour, const cv::Point2f &point)
{
    return cv::pointPolygonTest(contour, point, false) >= 0.0;
}

// 任一点到图像边界距离 <= margin 视为贴边。
bool near_image_border(const std::vector<cv::Point> &contour, const cv::Size &size, double margin)
{
    return std::any_of(contour.begin(), contour.end(),
                       [margin, &size](const cv::Point &point)
                       {
                           return point.x <= margin || point.y <= margin || point.x >= (size.width - 1) - margin ||
                                  point.y >= (size.height - 1) - margin;
                       });
}

double contour_solidity(const std::vector<cv::Point> &contour)
{
    const double area = cv::contourArea(contour);
    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);
    const double hull_area = cv::contourArea(hull);
    return hull_area > 0.0 ? (area / hull_area) : 0.0;
}

// 线段 begin->end 均匀采样,是否存在落在轮廓内的采样点。
bool line_pass_through_contour(const cv::Point2f &begin, const cv::Point2f &end, const std::vector<cv::Point> &contour,
                               int samples)
{
    if (samples <= 0)
    {
        return false;
    }
    for (int i = 0; i <= samples; ++i)
    {
        const float      t = static_cast<float>(i) / static_cast<float>(samples);
        const cv::Point2f point{begin.x + ((end.x - begin.x) * t), begin.y + ((end.y - begin.y) * t)};
        if (point_in_contour(contour, point))
        {
            return true;
        }
    }
    return false;
}

bool ellipse_descriptor_usable(const std::vector<cv::Point> &contour, const cv::Size &size, const RefinerConfig &config)
{
    return contour_solidity(contour) > config.solidity_threshold_ellipse &&
           !near_image_border(contour, size, config.border_margin);
}

bool rectangular_descriptor_usable(const std::vector<cv::Point> &contour, const RefinerConfig &config)
{
    if (contour_solidity(contour) <= config.solidity_threshold_rectangular)
    {
        return false;
    }
    const cv::RotatedRect rect       = cv::minAreaRect(contour);
    const double          long_side  = std::max(rect.size.width, rect.size.height);
    const double          short_side = std::min(rect.size.width, rect.size.height);
    if (short_side <= 0.0)
    {
        return false;
    }
    const double aspect_ratio   = long_side / short_side;
    const double relative_error = std::abs(aspect_ratio - config.expect_aspect_ratio) / config.expect_aspect_ratio;
    return relative_error < config.aspect_ratio_relative_error_threshold;
}

bool peak_descriptor_usable(const std::vector<cv::Point> &contour, const cv::Size &size, bool is_big,
                            const RefinerConfig &config)
{
    std::vector<cv::Point> approx;
    const double           epsilon = cv::arcLength(contour, true) * config.approx_error_tolerance;
    cv::approxPolyDP(contour, approx, epsilon, true);
    const int count = static_cast<int>(approx.size());
    if (is_big)
    {
        if (count < MIN_BIG_APPROX_POINTS || count > MAX_BIG_APPROX_POINTS)
        {
            return false;
        }
        return !near_image_border(contour, size, config.border_margin);
    }
    return count >= MIN_SMALL_APPROX_POINTS && count <= MAX_SMALL_APPROX_POINTS;
}

} // namespace

RefinerConfig load_refiner_config(const tools::config::Config &config)
{
    RefinerConfig refiner;
    refiner.red_minus_blue_threshold =
        config.value<int>("auto_buff.detector.refine.red_minus_blue_threshold", refiner.red_minus_blue_threshold);
    refiner.blue_minus_red_threshold =
        config.value<int>("auto_buff.detector.refine.blue_minus_red_threshold", refiner.blue_minus_red_threshold);
    refiner.armor_module_area_relative_error_threshold = config.value<double>(
        "auto_buff.detector.refine.armor_module_area_relative_error_threshold", refiner.armor_module_area_relative_error_threshold);
    refiner.light_arm_line_samples =
        config.value<int>("auto_buff.detector.refine.light_arm_line_samples", refiner.light_arm_line_samples);
    refiner.solidity_threshold_ellipse =
        config.value<double>("auto_buff.detector.refine.solidity_threshold_ellipse", refiner.solidity_threshold_ellipse);
    refiner.solidity_threshold_rectangular =
        config.value<double>("auto_buff.detector.refine.solidity_threshold_rectangular", refiner.solidity_threshold_rectangular);
    refiner.expect_aspect_ratio =
        config.value<double>("auto_buff.detector.refine.expect_aspect_ratio", refiner.expect_aspect_ratio);
    refiner.aspect_ratio_relative_error_threshold = config.value<double>(
        "auto_buff.detector.refine.aspect_ratio_relative_error_threshold", refiner.aspect_ratio_relative_error_threshold);
    refiner.approx_error_tolerance =
        config.value<double>("auto_buff.detector.refine.approx_error_tolerance", refiner.approx_error_tolerance);
    refiner.roi_margin_ratio = config.value<double>("auto_buff.detector.refine.roi_margin_ratio", refiner.roi_margin_ratio);
    refiner.border_margin    = config.value<double>("auto_buff.detector.refine.border_margin", refiner.border_margin);
    return refiner;
}

RuneRefinement refine(const cv::Mat &bgr, const Rune2d &rune, const RefinerConfig &config)
{
    RuneRefinement refinement;
    if (bgr.empty())
    {
        return refinement;
    }

    const cv::Point2f &top     = rune.keypoints[kpt_top];
    const cv::Point2f &left    = rune.keypoints[kpt_left];
    const cv::Point2f &right   = rune.keypoints[kpt_right];
    const cv::Point2f &bottom  = rune.keypoints[kpt_bottom];
    const cv::Point2f &point_r = rune.keypoints[kpt_r];

    // 4 个靶角点的外接框外扩成 ROI(代替深大的网络椭圆 view)。
    const float min_x = std::min({top.x, left.x, right.x, bottom.x});
    const float max_x = std::max({top.x, left.x, right.x, bottom.x});
    const float min_y = std::min({top.y, left.y, right.y, bottom.y});
    const float max_y = std::max({top.y, left.y, right.y, bottom.y});
    const float margin = static_cast<float>(config.roi_margin_ratio) * std::max(max_x - min_x, max_y - min_y);
    cv::Rect    roi(cv::Point(static_cast<int>(std::floor(min_x - margin)), static_cast<int>(std::floor(min_y - margin))),
                    cv::Point(static_cast<int>(std::ceil(max_x + margin)), static_cast<int>(std::ceil(max_y + margin))));
    roi &= cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (roi.width <= 0 || roi.height <= 0)
    {
        return refinement;
    }

    // 颜色差分二值化:红(R-B)/蓝(B-R)。
    const cv::Mat view = bgr(roi);
    std::vector<cv::Mat> channels;
    cv::split(view, channels);
    cv::Mat diff;
    if (rune.color == Color::red)
    {
        cv::subtract(channels[2], channels[0], diff);
    }
    else
    {
        cv::subtract(channels[0], channels[2], diff);
    }
    const int threshold_value =
        (rune.color == Color::red) ? config.red_minus_blue_threshold : config.blue_minus_red_threshold;
    cv::GaussianBlur(diff, diff, cv::Size(5, 5), 0.0);
    cv::Mat binary;
    cv::threshold(diff, binary, threshold_value, 255, cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    for (auto &contour : contours)
    {
        for (auto &point : contour)
        {
            point += roi.tl();
        }
    }

    const cv::Point2f center{(top.x + left.x + right.x + bottom.x) * 0.25F,
                             (top.y + left.y + right.y + bottom.y) * 0.25F};
    const double      nn_ellipse_area = 0.25 * cv::norm(top - bottom) * cv::norm(left - right) * PI;

    // 每类轮廓唯一:装甲板取面积误差最小,灯臂取面积最大,中心 R 取面积最小。
    double best_armor_error  = std::numeric_limits<double>::max();
    double best_light_area   = -1.0;
    double best_center_area  = std::numeric_limits<double>::max();
    for (const auto &contour : contours)
    {
        if (static_cast<int>(contour.size()) <= MIN_CONTOUR_POINTS)
        {
            continue;
        }
        const double area     = cv::contourArea(contour);
        const bool   center_in = point_in_contour(contour, center);
        const bool   r_in      = point_in_contour(contour, point_r);

        const bool is_armor = center_in && (nn_ellipse_area > 0.0) &&
                              (std::abs(area - nn_ellipse_area) / nn_ellipse_area <=
                               config.armor_module_area_relative_error_threshold);
        const bool is_light = !center_in && !r_in &&
                              line_pass_through_contour(center, point_r, contour, config.light_arm_line_samples) &&
                              !line_pass_through_contour(left, right, contour, config.light_arm_line_samples);
        const bool is_center_r = r_in && !center_in && !point_in_contour(contour, top) &&
                                 !point_in_contour(contour, left) && !point_in_contour(contour, right) &&
                                 !point_in_contour(contour, bottom);
        const int matched = (is_armor ? 1 : 0) + (is_light ? 1 : 0) + (is_center_r ? 1 : 0);
        if (matched > 1)
        {
            // 轮廓语义不唯一,放弃本帧(与深大 LOG(ERROR) 一致)。
            return RuneRefinement{};
        }
        if (is_armor)
        {
            const double error = std::abs(area - nn_ellipse_area) / nn_ellipse_area;
            if (error < best_armor_error)
            {
                best_armor_error       = error;
                refinement.armor_module = contour;
            }
        }
        else if (is_light)
        {
            if (area > best_light_area)
            {
                best_light_area       = area;
                refinement.light_arm = contour;
            }
        }
        else if (is_center_r)
        {
            if (area < best_center_area)
            {
                best_center_area       = area;
                refinement.center_r = contour;
            }
        }
    }

    refinement.is_armor_module_usable =
        !refinement.armor_module.empty() && ellipse_descriptor_usable(refinement.armor_module, bgr.size(), config);
    if (!refinement.light_arm.empty())
    {
        if (rune.kind == Kind::inactive)
        {
            refinement.is_light_arm_usable = rectangular_descriptor_usable(refinement.light_arm, config);
        }
        else
        {
            const bool is_big = (rune.kind == Kind::big_activated);
            refinement.is_light_arm_usable = peak_descriptor_usable(refinement.light_arm, bgr.size(), is_big, config);
        }
    }
    refinement.is_center_r_usable = !refinement.center_r.empty();
    return refinement;
}

} // namespace rm::buff
