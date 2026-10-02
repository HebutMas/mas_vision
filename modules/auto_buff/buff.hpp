#pragma once

#include <array>
#include <cstdint>

#include <opencv2/core/types.hpp>

namespace rm::buff
{

// 模型输出的能量机关状态类别,取值即类别索引。
enum class Kind : std::uint8_t
{
    inactive        = 0,
    small_activated = 1,
    big_activated   = 2,
};

// 5 个关键点的固定顺序(与模型训练/RP-26Rune 一致)。
// 索引 2 是 R 标(符心旋转中心),其余 4 个是符叶靶面的角点。
constexpr int KEYPOINT_COUNT = 5;

enum KeypointIndex : int
{
    KPT_TOP    = 0,
    KPT_LEFT   = 1,
    KPT_R      = 2,
    KPT_RIGHT  = 3,
    KPT_BOTTOM = 4,
};

// 能量机关阵营:由下发的 VisionMode 决定,模型本身不区分颜色。
enum class Color : std::uint8_t
{
    red  = 0,
    blue = 1,
};

// 单个能量机关检测结果。
struct Rune2d
{
    Kind                                    kind{Kind::inactive};         // 状态类别
    Color                                   color{Color::red};            // 阵营颜色
    float                                   confidence{0.0F};             // 类别置信度
    std::array<cv::Point2f, KEYPOINT_COUNT> keypoints{};                  // 5 个关键点(像素)
    std::array<float, KEYPOINT_COUNT>       keypoint_confidence{};        // 各关键点置信度
};

} // namespace rm::buff
