#pragma once

#include "modules/auto_armor/armor.hpp"
#include "modules/auto_armor/models/model.hpp"

#include <cstddef>
#include <optional>
#include <string>

#include <opencv2/core.hpp>

namespace rm::armor
{

// shenzhen-0526 / shenzhen-0708 两款 4 点 keypoint 模型的输出布局。
// 每条约 22 个 float:corners(8) + confidence(1) + color(4) + genre(9),两款模型一致。
struct ShenZhenResult
{
    using precision = float;

    struct Corners
    {
        precision lt_x, lt_y, lb_x, lb_y, rb_x, rb_y, rt_x, rt_y;
    } corners;

    precision confidence;

    struct Colors
    {
        precision blue, red, dark, mix;
    } color;

    struct Genres
    {
        precision sentry, hero, engineer, infantry_3, infantry_4, infantry_5, outpost, base_small, base_large;
    } genre;
};

static_assert(sizeof(ShenZhenResult) == 22 * sizeof(float), "shenzhen 输出必须是 22 个 float");

// 单条输出占用的 float 数。
inline constexpr std::size_t SHENZHEN_FLOATS = 22;

// 文件名是否属于受支持的 shenzhen 模型。
[[nodiscard]] bool is_shenzhen_model(const std::string &filename);

// shenzhen 模型的输出解析器,交给通用的 Detector 使用。
[[nodiscard]] const ModelSpec &shenzhen_model_spec();

// 解析单条模型输出为装甲板:
// - confidence 经 sigmoid 后需 > min_confidence,否则返回 nullopt;
// - color / genre 取 argmax;genre 为 UNKNOWN 时返回 nullopt;
// - 角点按 tl, tr, br, bl 输出,并做 letterbox 反算:p * inv_scale + offset。
[[nodiscard]] std::optional<Armor2d> decode_shenzhen(const ShenZhenResult &raw, float min_confidence, cv::Point2f offset, float inv_scale);

} // namespace rm::armor
