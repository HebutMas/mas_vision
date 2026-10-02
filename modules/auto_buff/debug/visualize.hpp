#pragma once

#include "modules/auto_buff/detection/detector.hpp"

#include <opencv2/core.hpp>

namespace rm::buff
{

// 把能量机关识别结果画到图上。
void draw(cv::Mat &image, const Detector::Result &result);

} // namespace rm::buff
