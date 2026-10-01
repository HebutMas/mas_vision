#include "modules/auto_armor/models/shenzhen_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace rm::armor
{
namespace
{

float sigmoid(float x) { return 1.0F / (1.0F + std::exp(-x)); }

template <std::size_t N> std::size_t argmax(const float (&values)[N])
{
    return static_cast<std::size_t>(std::max_element(values, values + N) - values);
}

// 把原始行指针还原成结构体再解析。
std::optional<Armor2d> decode_row(const float *row, float min_confidence, cv::Point2f offset, float inv_scale)
{
    ShenZhenResult raw;
    std::memcpy(&raw, row, sizeof(raw));
    return decode_shenzhen(raw, min_confidence, offset, inv_scale);
}

} // namespace

bool is_shenzhen_model(const std::string &filename) { return filename == "shenzhen-0526.onnx" || filename == "shenzhen-0708.onnx"; }

const ModelSpec &shenzhen_model_spec()
{
    static const ModelSpec spec{SHENZHEN_FLOATS, &decode_row};
    return spec;
}

std::optional<Armor2d> decode_shenzhen(const ShenZhenResult &raw, float min_confidence, cv::Point2f offset, float inv_scale)
{
    const float confidence = sigmoid(raw.confidence);
    if (confidence <= min_confidence)
    {
        return std::nullopt;
    }

    // 颜色顺序必须与模型输出一致 {blue, red, dark, mix}。
    const float     colors[4]{raw.color.blue, raw.color.red, raw.color.dark, raw.color.mix};
    constexpr Color COLORS[4]{Color::blue, Color::red, Color::gray, Color::purple};

    // 类别索引 0 对应 UNKNOWN(模型无此输出,恒为 0)。
    const float genres[10]{
        0.0F,
        raw.genre.hero,
        raw.genre.engineer,
        raw.genre.infantry_3,
        raw.genre.infantry_4,
        raw.genre.infantry_5,
        raw.genre.sentry,
        raw.genre.outpost,
        raw.genre.base_small,
        raw.genre.base_large,
    };
    constexpr Kind KINDS[10]{Kind::small,     Kind::hero,   Kind::engineer, Kind::infantry1, Kind::infantry2,
                             Kind::infantry3, Kind::sentry, Kind::outpost,  Kind::base,      Kind::base};

    const std::size_t genre = argmax(genres);
    if (genre == 0)
    {
        return std::nullopt;
    }

    Armor2d armor;
    armor.color      = COLORS[argmax(colors)];
    armor.kind       = KINDS[genre];
    armor.confidence = confidence;

    const cv::Point2f lt{raw.corners.lt_x, raw.corners.lt_y};
    const cv::Point2f tr{raw.corners.rt_x, raw.corners.rt_y};
    const cv::Point2f br{raw.corners.rb_x, raw.corners.rb_y};
    const cv::Point2f bl{raw.corners.lb_x, raw.corners.lb_y};
    armor.corners = {lt * inv_scale + offset, tr * inv_scale + offset, br * inv_scale + offset, bl * inv_scale + offset};
    armor.center  = (armor.corners[0] + armor.corners[1] + armor.corners[2] + armor.corners[3]) * 0.25F;
    return armor;
}

} // namespace rm::armor
