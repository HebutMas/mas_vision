#pragma once

#include <opencv2/core.hpp>

#include "modules/auto_armor/detection/detector.hpp"

namespace rm::armor
{

void draw(cv::Mat &image, const Detector::Result &detection);

} // namespace rm::armor
