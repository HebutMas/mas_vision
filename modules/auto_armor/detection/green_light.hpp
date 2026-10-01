#pragma once

#include "modules/auto_armor/armor.hpp"

#include <optional>
#include <vector>

#include <opencv2/core.hpp>

namespace rm::armor
{

// 绿灯(前哨站 / 基地顶部的绿色指示灯)滤除参数。
struct GreenLightConfig
{
    bool   enable{true};
    int    green_threshold{120};  // G 通道亮度下限
    double min_area{20.0};        // 绿灯最小面积
    double min_circularity{0.6};  // 最小圆度,绿灯为圆形
    double max_aspect_ratio{1.5}; // 最大长宽比
};

// 绿灯搜索结果;green_light 为全图坐标。
struct GreenLightResult
{
    std::optional<cv::Rect> roi;
    std::optional<cv::Rect> green_light;
};

// 在装甲板附近搜索绿灯。armors 只传建筑类(前哨站 / 基地)。
[[nodiscard]] GreenLightResult find_green_light(const cv::Mat &bgr, const std::vector<Armor2d> &armors, const GreenLightConfig &config);

// 删除位于绿灯上方的建筑类装甲板
void filter_buildings_above_green_light(std::vector<Armor2d> &armors, const cv::Rect &green_light);

} // namespace rm::armor
