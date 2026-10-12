#pragma once

#include "modules/auto_armor/armor.hpp"
#include "tools/config/config.hpp"

#include <optional>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

// 单板 PnP
namespace rm::armor
{

// 装甲板物理尺寸(毫米):四角即灯条端点,width为两灯条间距,height为灯条发光长
struct ArmorGeometry
{
    double small_width{135.0}, small_height{55.0};
    double large_width{225.0}, large_height{55.0};
};

struct SolverConfig
{
    cv::Mat       camera_matrix; // 3x3,CV_64F,像素
    cv::Mat       distort_coeff; // 1xN,CV_64F
    ArmorGeometry geometry;
};

// 从配置读取相机内参与畸变
[[nodiscard]] SolverConfig load_solver_config(const tools::config::Config &config);

// 单块装甲板 PnP:2D 角点 -> 云台系 Armor3d(x 前、y 左、z 上)。
class Solver
{
  public:
    explicit Solver(SolverConfig config);

    [[nodiscard]] std::optional<Armor3d> solve(const Armor2d &armor) const;

  private:
    SolverConfig             config_;
    std::vector<cv::Point3f> small_points_;
    std::vector<cv::Point3f> large_points_;
};

} // namespace rm::armor
