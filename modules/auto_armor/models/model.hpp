#pragma once

#include "modules/auto_armor/armor.hpp"

#include <cstddef>
#include <optional>
#include <string>

#include <opencv2/core.hpp>

namespace rm::armor
{

// 模型输出解析器:把单行原始输出(float)解码成一块装甲板。
// 每个模型在自己的 hpp/cpp 里给出 floats_per_detection 和 decode;
struct ModelSpec
{
    std::size_t floats_per_detection;
    std::optional<Armor2d> (*decode)(const float *row, float min_confidence, cv::Point2f offset, float inv_scale);
};

// 按模型文件名查找输出解析器;未支持的模型返回 nullptr。
[[nodiscard]] const ModelSpec *find_model_spec(const std::string &filename);

} // namespace rm::armor
