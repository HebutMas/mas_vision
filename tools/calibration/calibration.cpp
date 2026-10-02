#include "tools/calibration/calibration.hpp"

#include <opencv2/calib3d.hpp>

#include <cstddef>
#include <stdexcept>
#include <string>

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

} // namespace tools::calibration
