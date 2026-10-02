#pragma once

#include "modules/auto_buff/buff.hpp"
#include "tools/config/config.hpp"

#include <opencv2/core.hpp>

namespace rm::buff
{

// 识别后精修(移植自深大 RP-26Rune 的 RuneObservationRefiner 精简版):
// 网络五点提供语义标签,颜色差分二值轮廓提供像素精度,按包含关系把轮廓
// 分类为装甲板模块 / 灯臂 / 中心 R 标,再用形状描述符判断各特征是否可用。
struct RefinerConfig
{
    // 颜色差分二值化阈值(红: R-B, 蓝: B-R)。
    int red_minus_blue_threshold{60};
    int blue_minus_red_threshold{62};
    // 候选装甲板轮廓面积相对网络框面积的最大误差。
    double armor_module_area_relative_error_threshold{0.35};
    // 判断灯臂连线是否穿过轮廓时的采样点数。
    int light_arm_line_samples{30};
    // 装甲板(椭圆)轮廓最小 solidity = 轮廓面积 / 凸包面积。
    double solidity_threshold_ellipse{0.8};
    // 未激活灯臂(矩形)轮廓最小 solidity。
    double solidity_threshold_rectangular{0.66};
    // 未激活灯臂期望长宽比及允许相对误差。
    double expect_aspect_ratio{5.0};
    double aspect_ratio_relative_error_threshold{0.42};
    // 已激活灯臂多边形近似的相对容差。
    double approx_error_tolerance{0.01};
    // ROI 相对网络五点外接框的外扩比例。
    double roi_margin_ratio{0.15};
    // 轮廓贴边判定:点到图像边界的距离(像素)。
    double border_margin{2.0};
};

[[nodiscard]] RefinerConfig load_refiner_config(const tools::config::Config &config);

// 对单个检测结果做精修;rune.keypoints 为原图坐标,结果写入 Rune2d::refinement。
[[nodiscard]] RuneRefinement refine(const cv::Mat &bgr, const Rune2d &rune, const RefinerConfig &config);

} // namespace rm::buff
