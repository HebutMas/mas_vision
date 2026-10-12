#include "modules/auto_armor/track/solver.hpp"

#include "tools/algorithm/math_tools.hpp"

#include <cmath>
#include <utility>

#include <opencv2/calib3d.hpp>

namespace rm::armor
{

namespace
{
// 物点:装甲板本体系(x 前=法线,y 左,z 上),顺序tl,tr,br,bl,单位米。
std::vector<cv::Point3f> make_object_points(double width, double height)
{
    const float hy = static_cast<float>(width) / 2.0F / 1000.0F;
    const float hz = static_cast<float>(height) / 2.0F / 1000.0F;
    return {{0.0F, hy, hz}, {0.0F, -hy, hz}, {0.0F, -hy, -hz}, {0.0F, hy, -hz}};
}
} // namespace

SolverConfig load_solver_config(const tools::config::Config &config)
{
    SolverConfig out;
    const auto   k    = config.require<std::vector<double>>("hikcamera.camera_matrix");
    auto         d    = config.value<std::vector<double>>("hikcamera.distort_coeffs", {});
    out.camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double *>(k.data())).clone();
    out.distort_coeff = cv::Mat(1, static_cast<int>(d.size()), CV_64F, const_cast<double *>(d.data())).clone();

    return out;
}

Solver::Solver(SolverConfig config) : config_(std::move(config))
{
    small_points_ = make_object_points(config_.geometry.small_width, config_.geometry.small_height);
    large_points_ = make_object_points(config_.geometry.large_width, config_.geometry.large_height);
}

std::optional<Armor3d> Solver::solve(const Armor2d &armor) const
{
    const bool                     large = (armor.kind == Kind::hero);
    const auto                    &obj   = large ? large_points_ : small_points_;
    const std::vector<cv::Point2f> img{armor.corners[0], armor.corners[1], armor.corners[2], armor.corners[3]};

    cv::Mat rvec;
    cv::Mat tvec;
    if (!cv::solvePnP(obj, img, config_.camera_matrix, config_.distort_coeff, rvec, tvec, false, cv::SOLVEPNP_IPPE))
    {
        return std::nullopt;
    }

    cv::Mat rot;
    cv::Rodrigues(rvec, rot);
    // 相机系(OpenCV:x 右,y 下,z 前) -> 云台系(x 前,y 左,z 上):gx=cz, gy=-cx, gz=-cy。
    Armor3d out;
    out.kind       = armor.kind;
    out.color      = armor.color;
    out.confidence = armor.confidence;
    out.xyz        = Eigen::Vector3d(tvec.at<double>(2), -tvec.at<double>(0), -tvec.at<double>(1));
    // 板法线 = 本体 x 轴在相机系下的像 = R 第一列。矩形板 PnP 有法线正反二义,只可能看到正面,
    // 故强制法线朝向相机(法线·位置 < 0);再转云台系,方位角即朝向,模型里 ψ = 方位角 - π。
    Eigen::Vector3d       normal(rot.at<double>(0, 0), rot.at<double>(1, 0), rot.at<double>(2, 0));
    const Eigen::Vector3d t_cam(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    if (normal.dot(t_cam) > 0.0) normal = -normal;
    const Eigen::Vector3d n_gimbal(normal.z(), -normal.x(), -normal.y());
    out.yaw = tools::limit_rad(std::atan2(n_gimbal.y(), n_gimbal.x()) - M_PI);
    return out;
}

} // namespace rm::armor
