#pragma once

#include <array>
#include <cstdint>
#include <vector>

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

enum KeypointIndex : std::uint8_t
{
    kpt_top    = 0,
    kpt_left   = 1,
    kpt_r      = 2,
    kpt_right  = 3,
    kpt_bottom = 4,
};

// 能量机关阵营:由下发的 VisionMode 决定,模型本身不区分颜色。
enum class Color : std::uint8_t
{
    red  = 0,
    blue = 1,
};

// 状态类别 -> 可读名称(可视化与日志用)
[[nodiscard]] inline const char *kind_name(Kind kind)
{
    switch (kind)
    {
    case Kind::inactive:
        return "inactive";
    case Kind::small_activated:
        return "small_activated";
    case Kind::big_activated:
        return "big_activated";
    }
    return "unknown";
}

// 阵营颜色 -> 可读名称(可视化与日志用)。
[[nodiscard]] inline const char *color_name(Color color)
{
    switch (color)
    {
    case Color::red:
        return "red";
    case Color::blue:
        return "blue";
    }
    return "unknown";
}

// 单个符叶的识别后精修结果(移植自深大 RP-26Rune 精简版)。
// 轮廓为空表示该特征未找到。
struct RuneRefinement
{
    std::vector<cv::Point> armor_module; // 装甲板模块轮廓
    std::vector<cv::Point> light_arm;    // 灯臂轮廓
    std::vector<cv::Point> center_r;     // 中心 R 标轮廓
    bool                   is_armor_module_usable{false};
    bool                   is_light_arm_usable{false};
    bool                   is_center_r_usable{false};
};

// 单个能量机关检测结果。
struct Rune2d
{
    Kind                                    kind{Kind::inactive};         // 状态类别
    Color                                   color{Color::red};            // 阵营颜色
    float                                   confidence{0.0F};             // 类别置信度
    std::array<cv::Point2f, KEYPOINT_COUNT> keypoints{};                  // 5 个关键点(像素)
    std::array<float, KEYPOINT_COUNT>       keypoint_confidence{};        // 各关键点置信度
    RuneRefinement                          refinement;                   // 识别后精修结果
};

} // namespace rm::buff
