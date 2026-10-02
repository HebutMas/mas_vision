#pragma once

#include "modules/auto_armor/detection/detector.hpp"

#include <opencv2/core.hpp>

namespace rm::armor
{

// 把识别结果画到图上。
void draw(cv::Mat &image, const Detector::Result &result);

} // namespace rm::armor
