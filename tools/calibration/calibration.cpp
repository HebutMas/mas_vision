#include "tools/calibration/calibration.hpp"

#include <Eigen/Geometry>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace tools::calibration
{

std::vector<cv::Point3f> BoardConfig::object_points() const
{
    if (rows <= 0 || cols <= 0)
    {
        throw std::invalid_argument("board rows/cols must be positive");
    }

    std::vector<cv::Point3f> points;
    points.reserve(static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols));
    for (int row = 0; row < rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            points.emplace_back(static_cast<float>(col) * static_cast<float>(square_size), static_cast<float>(row) * static_cast<float>(square_size),
                                0.0F);
        }
    }
    return points;
}

IntrinsicResult calibrate_intrinsic(const std::vector<std::vector<cv::Point2f>> &image_points, const BoardConfig &board, const cv::Size &image_size)
{
    // 平面靶标至少要几个不同姿态;太少直接报错,避免解出离谱内参。
    if (image_points.size() < 4)
    {
        throw std::invalid_argument("calibrate_intrinsic needs at least 4 views, got " + std::to_string(image_points.size()));
    }

    const std::vector<cv::Point3f>              object_points = board.object_points();
    const std::vector<std::vector<cv::Point3f>> object_points_all(image_points.size(), object_points);

    IntrinsicResult      result;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    cv::calibrateCamera(object_points_all, image_points, image_size, result.camera_matrix, result.distort_coeff, rvecs, tvecs);

    // 平均重投影误差:把棋盘格角点按标定结果投回像素,和检测值比。
    double      total = 0.0;
    std::size_t count = 0;
    for (std::size_t i = 0; i < image_points.size(); ++i)
    {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(object_points, rvecs[i], tvecs[i], result.camera_matrix, result.distort_coeff, projected);
        for (std::size_t j = 0; j < projected.size(); ++j)
        {
            total += cv::norm(projected[j] - image_points[i][j]);
            ++count;
        }
    }
    if (count != 0)
    {
        result.reprojection_error = total / static_cast<double>(count);
    }
    return result;
}

HandEyeResult calibrate_hand_eye(const std::vector<HandEyeSample> &samples)
{
    // 手眼标定靠不同姿态之间的旋转约束,太少或无转动都解不出来。
    if (samples.size() < 3)
    {
        throw std::invalid_argument("calibrate_hand_eye needs at least 3 samples, got " + std::to_string(samples.size()));
    }

    std::vector<cv::Mat> r_gimbal2world_list;
    std::vector<cv::Mat> t_gimbal2world_list;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    for (const auto &sample : samples)
    {
        r_gimbal2world_list.push_back(sample.r_gimbal2world);
        t_gimbal2world_list.push_back(sample.t_gimbal2world);
        rvecs.push_back(sample.rvec);
        tvecs.push_back(sample.tvec);
    }

    HandEyeResult result;
    cv::calibrateHandEye(r_gimbal2world_list, t_gimbal2world_list, rvecs, tvecs, result.r_camera2gimbal, result.t_camera2gimbal,
                         cv::CALIB_HAND_EYE_TSAI);

    // 一致性自检:各组 r_gimbal2world * r_camera2gimbal * r_board2camera 都应等于 r_board2world,
    // 取两两夹角的最大值当偏差。角度太集中时它会明显变大,提醒重新采数据。
    Eigen::Matrix3d r_camera2gimbal;
    cv::cv2eigen(result.r_camera2gimbal, r_camera2gimbal);

    std::vector<Eigen::Matrix3d> r_board2world;
    r_board2world.reserve(samples.size());
    for (const auto &sample : samples)
    {
        Eigen::Matrix3d r_gimbal2world;
        cv::cv2eigen(sample.r_gimbal2world, r_gimbal2world);

        cv::Mat r_board2camera_cv;
        cv::Rodrigues(sample.rvec, r_board2camera_cv);
        Eigen::Matrix3d r_board2camera;
        cv::cv2eigen(r_board2camera_cv, r_board2camera);

        r_board2world.emplace_back(r_gimbal2world * r_camera2gimbal * r_board2camera);
    }

    double max_deviation = 0.0;
    for (std::size_t i = 0; i < r_board2world.size(); ++i)
    {
        for (std::size_t j = i + 1; j < r_board2world.size(); ++j)
        {
            max_deviation = std::max(max_deviation, Eigen::AngleAxisd(r_board2world[i].transpose() * r_board2world[j]).angle());
        }
    }
    result.consistency_deg = max_deviation * 180.0 / M_PI;
    return result;
}

} // namespace tools::calibration
