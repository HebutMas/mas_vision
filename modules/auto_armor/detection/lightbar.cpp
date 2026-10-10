#include "modules/auto_armor/detection/lightbar.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace rm::armor
{
namespace
{

constexpr double K_ROI_MARGIN_RATIO = 0.15; // 灰度 ROI 的外扩比例(相对灯条长度)
constexpr double K_MIN_LENGTH       = 4.0;  // 灯条短于它就不精修

} // namespace

// 灯条 PCA 优化
void refine_lightbar(cv::Point2f &upper, cv::Point2f &lower, const cv::Mat &bgr, const LightRefineParams &params)
{
    if (bgr.empty() || (bgr.type() != CV_8UC3))
    {
        return;
    }

    const cv::Point2f delta  = upper - lower;
    const auto        length = cv::norm(delta);
    if (length < K_MIN_LENGTH)
    {
        return;
    }

    // 轴统一"朝上"(y 分量 <= 0),这样 top 就是 upper 那一侧
    cv::Point2f axis = delta * (1.0F / static_cast<float>(length));
    if (axis.y > 0.0F)
    {
        axis = -axis;
    }
    const cv::Point2f perpendicular{-axis.y, axis.x};

    // 对灯条附近 ROI 转灰度
    const int      margin = std::max(2, static_cast<int>(std::lround(K_ROI_MARGIN_RATIO * length)));
    const cv::Rect box    = (cv::boundingRect(std::vector<cv::Point2f>{upper, lower}) + cv::Size(2 * margin, 2 * margin)) - cv::Point(margin, margin);
    const cv::Rect roi    = box & cv::Rect{0, 0, bgr.cols, bgr.rows};
    if ((roi.width < 3) || (roi.height < 3))
    {
        return;
    }

    cv::Mat gray;
    cv::cvtColor(bgr(roi), gray, cv::COLOR_BGR2GRAY);

    const cv::Point2f offset{static_cast<float>(roi.x), static_cast<float>(roi.y)};
    const double      mean_value = cv::mean(gray)[0];

    const auto in_image = [&gray](const cv::Point2f &point) noexcept {
        return point.x >= 0.0F && point.x < static_cast<float>(gray.cols) && point.y >= 0.0F && point.y < static_cast<float>(gray.rows);
    };
    const auto brightness = [&gray](const cv::Point2f &point) noexcept {
        return static_cast<double>(gray.at<std::uint8_t>(cvRound(point.y), cvRound(point.x)));
    };

    // 亮度加权质心 + 协方差主轴
    double min_value = 0.0;
    cv::minMaxLoc(gray, &min_value, nullptr);

    double weight_sum = 0.0;
    double x_sum      = 0.0;
    double y_sum      = 0.0;
    double xx_sum     = 0.0;
    double yy_sum     = 0.0;
    double xy_sum     = 0.0;
    for (int row = 0; row < gray.rows; ++row)
    {
        for (int column = 0; column < gray.cols; ++column)
        {
            const auto weight = static_cast<double>(gray.at<std::uint8_t>(row, column)) - min_value;
            const auto x      = static_cast<double>(column);
            const auto y      = static_cast<double>(row);
            weight_sum += weight;
            x_sum += weight * x;
            y_sum += weight * y;
            xx_sum += weight * x * x;
            yy_sum += weight * y * y;
            xy_sum += weight * x * y;
        }
    }
    if (weight_sum <= 0.0)
    {
        return;
    }

    const double      center_x = x_sum / weight_sum;
    const double      center_y = y_sum / weight_sum;
    const cv::Point2f centroid{static_cast<float>(center_x), static_cast<float>(center_y)};

    // 平行轴公式直接从二阶矩得到协方差,省掉第二趟遍历;主轴 theta = 0.5·atan2(2·cov_xy, cov_xx − cov_yy)
    const double cov_xx = (xx_sum / weight_sum) - (center_x * center_x);
    const double cov_yy = (yy_sum / weight_sum) - (center_y * center_y);
    const double cov_xy = (xy_sum / weight_sum) - (center_x * center_y);
    const double theta  = 0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);
    cv::Point2f  direction{static_cast<float>(std::cos(theta)), static_cast<float>(std::sin(theta))};
    if (const float norm = cv::norm(direction); norm > 0.0F)
    {
        direction /= norm;
    }
    else
    {
        direction = axis;
    }
    if (direction.y > 0.0F)
    {
        direction = -direction;
    }

    // 灯条粗细:沿法向从质心往外扫,两侧都暗下去了就停
    int width = 1;
    {
        const int max_half = std::max(3, static_cast<int>(std::lround(0.25 * length)));
        for (int distance = 1; distance <= max_half; ++distance)
        {
            const cv::Point2f positive = centroid + (perpendicular * static_cast<float>(distance));
            const cv::Point2f negative = centroid - (perpendicular * static_cast<float>(distance));
            if (!in_image(positive) || !in_image(negative) || (brightness(positive) < mean_value && brightness(negative) < mean_value))
            {
                break;
            }
            width = (2 * distance) + 1;
        }
    }
    if (static_cast<double>(width) <= params.refine_min_width)
    {
        return;
    }

    // 沿轴搜端点:横向取 width−2 根采样,每根找"最大正向亮度落差"(且前一点亮于 ROI 均值),再平均
    const auto find_corner = [&](bool top) {
        const int   half_n = static_cast<int>(std::lround(std::max(1, width - 2) / 2.0));
        const float sign   = top ? 1.0F : -1.0F;
        const float dx     = direction.x * sign;
        const float dy     = direction.y * sign;
        const auto  steps  = static_cast<int>(std::floor(length * (params.refine_end_ratio - params.refine_start_ratio)));

        cv::Point2f sum{0.0F, 0.0F};
        int         found_count = 0;
        for (int lateral = -half_n; lateral <= half_n; ++lateral)
        {
            const auto        length_start = static_cast<float>(length * params.refine_start_ratio);
            const cv::Point2f start{centroid.x + (length_start * dx) + static_cast<float>(lateral), centroid.y + (length_start * dy)};

            cv::Point2f previous = start;
            cv::Point2f corner   = start;
            double      max_drop = 0.0;
            bool        found    = false;

            for (int step = 1; step <= steps; ++step)
            {
                const auto        step_offset = static_cast<float>(step);
                const cv::Point2f current{start.x + (dx * step_offset), start.y + (dy * step_offset)};
                if (!in_image(current))
                {
                    break;
                }
                const double drop = brightness(previous) - brightness(current);
                if ((drop > max_drop) && (brightness(previous) > mean_value))
                {
                    max_drop = drop;
                    corner   = previous;
                    found    = true;
                }
                previous = current;
            }

            if (found)
            {
                sum += corner;
                ++found_count;
            }
        }
        return found_count > 0 ? sum / static_cast<float>(found_count) : cv::Point2f{-1.0F, -1.0F};
    };

    const cv::Point2f refined_upper = find_corner(true);
    const cv::Point2f refined_lower = find_corner(false);
    if (refined_upper.x > 0.0F)
    {
        upper = refined_upper + offset;
    }
    if (refined_lower.x > 0.0F)
    {
        lower = refined_lower + offset;
    }
}

} // namespace rm::armor
