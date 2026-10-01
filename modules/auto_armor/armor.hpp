#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core/types.hpp>

namespace rm::armor
{

// 装甲板颜色。模型输出为 {blue, red, dark, mix} 四类,这里映射成语义色:
// dark -> 灰,dark/mix 用于灰板 / 紫色特殊灯色。
enum class Color : std::uint8_t
{
    red,
    blue,
    gray,
    purple,
};

// 装甲板类别。small / large 是通用大小板,其余是具体兵种。
enum class Kind : std::uint8_t
{
    small,
    large,
    outpost,
    base,
    sentry,
    hero,
    engineer,
    infantry1,
    infantry2,
    infantry3,
};

// 2D 装甲板,corners 顺序固定为 tl, tr, br, bl(像素坐标)。
struct Armor2d
{
    std::array<cv::Point2f, 4> corners;
    cv::Point2f                center;
    Kind                       kind{Kind::small};
    Color                      color{Color::blue};
    float                      confidence{0.0F};
};

// 2D 灯条。
struct Lightbar2d
{
    Kind        kind{Kind::small};
    Color       color{Color::blue};
    cv::Point2f upper;
    cv::Point2f lower;
};

// 把配置字符串("red"/"blue"/"gray"/"purple")解析成颜色;无法识别返回 nullopt。
[[nodiscard]] inline std::optional<Color> parse_color(const std::string &name)
{
    if (name == "red") return Color::red;
    if (name == "blue") return Color::blue;
    if (name == "gray") return Color::gray;
    if (name == "purple") return Color::purple;
    return std::nullopt;
}

// 只保留指定颜色的装甲板
inline void filter_by_color(std::vector<Armor2d> &armors, Color color)
{
    armors.erase(std::remove_if(armors.begin(), armors.end(), [color](const Armor2d &armor) { return armor.color != color; }), armors.end());
}

} // namespace rm::armor
