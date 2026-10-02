#pragma once

#include "modules/auto_buff/buff.hpp"
#include "modules/auto_buff/models/model.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace rm::buff
{

// shenzhenbuff-0624:YOLOv8n-pose 五点模型,每个 anchor 18 个通道
// = 3 类别 + 5 关键点 × (x, y, conf)。
inline constexpr std::size_t SHENZHENBUFF_CHANNELS = 18;

// 文件名是否属于受支持的深大能量机关模型。
[[nodiscard]] bool is_shenzhenbuff(const std::string &filename);

// 深大能量机关模型的输出解析器,交给通用的 Detector 使用。
[[nodiscard]] const ModelSpec &shenzhenbuff_spec();

// 解析整个输出张量:
// 每 anchor 取类别 argmax,需 > confidence_threshold;
// 解码 5 个关键点,按 crop_offset / inv_scale 反映射到原图并截断;
// 有效关键点数 < min_valid_keypoints 的 anchor 丢弃。
void decode_shenzhenbuff(const ModelOutput &output, const DecodeParams &params, std::vector<Rune2d> &out);

} // namespace rm::buff
