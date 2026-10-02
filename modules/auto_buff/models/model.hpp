#pragma once

#include "modules/auto_buff/buff.hpp"

#include <cstddef>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace rm::buff
{

// 网络原始输出视图:channel_first 为 true 时 data[c * anchors + a],否则 data[a * channels + c]。
struct ModelOutput
{
    const float *data;
    std::size_t  channels;
    std::size_t  anchors;
    bool         channel_first;
};

// 把模型输入坐标还原到原图,以及后处理阈值。
struct DecodeParams
{
    cv::Point2f crop_offset;               // 居中裁剪的原点(原图像素)
    float       inv_scale_x;               // 模型输入 -> 原图 x 方向缩放倒数
    float       inv_scale_y;               // 模型输入 -> 原图 y 方向缩放倒数
    float       confidence_threshold;      // 类别置信度阈值
    float       keypoint_confidence_threshold; // 单关键点置信度阈值
    int         min_valid_keypoints;       // 有效关键点少于此数则丢弃
    cv::Size    image_size;                // 原图尺寸,用于关键点截断
};

// 模型输出解析器:每个模型在自己的 hpp/cpp 里给出 channels 与 decode。
struct ModelSpec
{
    std::size_t channels;
    void (*decode)(const ModelOutput &output, const DecodeParams &params, std::vector<Rune2d> &out);
};

// 按模型文件名查找输出解析器;未支持的模型返回 nullptr。
[[nodiscard]] const ModelSpec *find_model_spec(const std::string &filename);

} // namespace rm::buff
