#pragma once

#include "modules/auto_armor/armor.hpp"

#include <opencv2/core.hpp>

namespace rm::armor
{

// 灯条角点优化参数
struct LightRefineParams
{
    double refine_min_width{3.0};   // 最小灯条宽度(比它窄就不精修)
    double refine_start_ratio{0.4}; // 从质心沿对称轴 0.4L 处开始搜
    double refine_end_ratio{0.6};   // 搜到 0.6L;窗口 = (end - start)·L
};

// 灯条角点优化
void refine_lightbar(cv::Point2f &upper, cv::Point2f &lower, const cv::Mat &bgr, const LightRefineParams &params);

} // namespace rm::armor
