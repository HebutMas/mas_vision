#include "modules/auto_armor/debug/visualize.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstddef>
#include <string>

namespace rm::armor
{
namespace
{

// Canvas调色板
[[nodiscard]] cv::Scalar shape_color() noexcept { return {127, 127, 127}; }   // 灰:装甲板框
[[nodiscard]] cv::Scalar corner_color() noexcept { return {0, 255, 255}; }   // 黄:角点
[[nodiscard]] cv::Scalar label_color() noexcept { return {0, 255, 255}; }   // 黄:兵种标签
[[nodiscard]] cv::Scalar white_color() noexcept { return {255, 255, 255}; }  // 白:灯条/计数
[[nodiscard]] cv::Scalar red_color() noexcept { return {0, 0, 255}; }        // 红:光学中心十字

// 文本绘制
void draw_text(cv::Mat &image, const std::string &text, cv::Point origin, const cv::Scalar &color, double scale, int thickness)
{
    cv::putText(image, text, origin + cv::Point(1, 1), cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), thickness + 2, cv::LINE_AA);
    cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA);
}

} // namespace

void draw(cv::Mat &image, const Detector::Result &detection)
{
    if (image.empty())
    {
        return;
    }

    // 字号/线宽随分辨率放大(基准 0.4 / 1,与 rmcs 一致)。
    const double font_scale = std::max(0.4, image.rows / 1080.0);
    const int    thickness  = 1 + (image.rows / 1080);
    const int    text_x     = cvRound(10.0 * font_scale);
    const int    text_y     = image.rows - cvRound(90.0 * font_scale);

    // 光学中心十字(红)。
    const auto  half = static_cast<float>(12.0 * font_scale);
    const float ox   = static_cast<float>(image.cols) / 2.0F;
    const float oy   = static_cast<float>(image.rows) / 2.0F;
    cv::line(image, {cvRound(ox - half), cvRound(oy)}, {cvRound(ox + half), cvRound(oy)}, red_color(), thickness, cv::LINE_AA);
    cv::line(image, {cvRound(ox), cvRound(oy - half)}, {cvRound(ox), cvRound(oy + half)}, red_color(), thickness, cv::LINE_AA);

    // 每块 2D 装甲板:灰框 + 角点圆点 + 兵种/颜色/置信度标签;
    // 对应两根灯条画白线(灯条数 = 2 x 板数,顺序与 corners 的 tl/tr 一致)。
    const int dot_radius = 2 + (image.rows / 1080);
    for (std::size_t index = 0; index < detection.armors.size(); ++index)
    {
        const auto &armor = detection.armors[index];

        for (std::size_t corner_index = 0; corner_index < armor.corners.size(); ++corner_index)
        {
            const cv::Point2f from = armor.corners[corner_index];
            const cv::Point2f to   = armor.corners[(corner_index + 1) % armor.corners.size()];
            cv::line(image, from, to, shape_color(), thickness, cv::LINE_AA);
            cv::circle(image, from, dot_radius, corner_color(), cv::FILLED, cv::LINE_AA);
        }

        if ((2 * index) + 1 < detection.lightbars.size())
        {
            for (std::size_t bar = (2 * index); bar < (2 * index) + 2; ++bar)
            {
                cv::line(image, detection.lightbars[bar].upper, detection.lightbars[bar].lower, white_color(), thickness, cv::LINE_AA);
            }
        }

        // 标签挂在最高角点上方。
        cv::Point2f top = armor.corners.front();
        for (const auto &corner : armor.corners)
        {
            if (corner.y < top.y)
            {
                top = corner;
            }
        }

        const cv::Point  label_origin{cvRound(top.x), cvRound(top.y - (15.0 * font_scale))};
        const std::string label =
            std::string(kind_name(armor.kind)) + " " + color_name(armor.color) + " " + cv::format("%.2f", armor.confidence);
        draw_text(image, label, label_origin, label_color(), 0.5 * font_scale, thickness);
    }

    // 左下角计数面板。
    draw_text(image, cv::format("armor %d", static_cast<int>(detection.armors.size())), {text_x, text_y}, white_color(), 0.6 * font_scale, thickness);
}

} // namespace rm::armor
