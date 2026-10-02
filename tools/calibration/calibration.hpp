#pragma once

#include <opencv2/core.hpp>

#include <vector>

namespace tools::calibration
{

// 棋盘格规格:rows / cols 是内角点数(不是方格数),square_size 是方格边长(米)。
struct BoardConfig
{
    int    rows{11};            // 内角点行数
    int    cols{8};             // 内角点列数
    double square_size{0.0181}; // 方格边长(米)

    // findChessboardCorners 的 patternSize:(每行角点数, 每列角点数)。
    [[nodiscard]] cv::Size size() const noexcept { return {cols, rows}; }

    // 角点在棋盘系下的坐标,行优先:index = row * cols + col,与 OpenCV 检测顺序一致。
    [[nodiscard]] std::vector<cv::Point3f> object_points() const;
};

// 内参标定结果(张正友法)。
struct IntrinsicResult
{
    cv::Mat camera_matrix;           // 3x3,CV_64F
    cv::Mat distort_coeff;           // 1xN,CV_64F
    double  reprojection_error{0.0}; // 平均重投影误差(像素)
};

// 用多张棋盘格角点标定内参。image_points[i] 是第 i 张图的角点,顺序与 board.object_points() 一致。
[[nodiscard]] IntrinsicResult calibrate_intrinsic(const std::vector<std::vector<cv::Point2f>> &image_points, const BoardConfig &board,
                                                  const cv::Size &image_size);

} // namespace tools::calibration
