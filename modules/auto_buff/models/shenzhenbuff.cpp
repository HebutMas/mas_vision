#include "modules/auto_buff/models/shenzhenbuff.hpp"

#include <algorithm>

namespace rm::buff
{
namespace
{

// 类别数,与模型输出的前 CLASS_COUNT 个通道对应。
constexpr int CLASS_COUNT = 3;

} // namespace

bool is_shenzhenbuff(const std::string &filename) { return filename.rfind("shenzhenbuff", 0) == 0; }

const ModelSpec &shenzhenbuff_spec()
{
    static const ModelSpec spec{SHENZHENBUFF_CHANNELS, &decode_shenzhenbuff};
    return spec;
}

void decode_shenzhenbuff(const ModelOutput &output, const DecodeParams &params, std::vector<Rune2d> &out)
{
    const auto value = [&](std::size_t channel, std::size_t anchor) {
        return output.channel_first ? output.data[(channel * output.anchors) + anchor] : output.data[(anchor * output.channels) + channel];
    };

    for (std::size_t anchor = 0; anchor < output.anchors; ++anchor)
    {
        int   best_class = -1;
        float best_score = params.confidence_threshold;
        for (int c = 0; c < CLASS_COUNT; ++c)
        {
            const float score = value(static_cast<std::size_t>(c), anchor);
            if (score > best_score)
            {
                best_score = score;
                best_class = c;
            }
        }
        if (best_class < 0)
        {
            continue;
        }

        Rune2d rune;
        rune.kind       = static_cast<Kind>(best_class);
        rune.confidence = best_score;

        bool valid      = true;
        int  valid_kpts = 0;
        for (int k = 0; k < KEYPOINT_COUNT; ++k)
        {
            const auto  base  = static_cast<std::size_t>(CLASS_COUNT) + (static_cast<std::size_t>(k) * 3U);
            const float x     = (value(base, anchor) * params.inv_scale_x) + params.crop_offset.x;
            const float y     = (value(base + 1, anchor) * params.inv_scale_y) + params.crop_offset.y;
            const float kconf = value(base + 2, anchor);
            if (x < 0.0F || y < 0.0F)
            {
                valid = false;
                break;
            }
            rune.keypoints[k] = {std::min(x, static_cast<float>(params.image_size.width - 1)),
                                 std::min(y, static_cast<float>(params.image_size.height - 1))};
            rune.keypoint_confidence[k] = kconf;
            if (kconf >= params.keypoint_confidence_threshold)
            {
                ++valid_kpts;
            }
        }
        if (!valid || valid_kpts < params.min_valid_keypoints)
        {
            continue;
        }

        out.push_back(rune);
    }
}

} // namespace rm::buff
