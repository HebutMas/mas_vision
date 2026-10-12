#pragma once

#include <optional>

#include <opencv2/core.hpp>

#include "modules/auto_armor/track/solver.hpp"
#include "modules/auto_armor/track/target.hpp"

namespace rm::armor
{

// 把识别结果画到图上。
void draw(cv::Mat &image, const std::optional<track::TargetState> &state, const SolverConfig &solver);

} // namespace rm::armor
