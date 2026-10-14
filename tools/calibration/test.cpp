#include "tools/calibration/calibration.hpp"

#include <Eigen/Geometry>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <cmath>
#include <exception>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
int failures = 0;

void check(bool ok, const std::string &what)
{
    if (!ok)
    {
        std::cerr << "FAIL: " << what << '\n';
        ++failures;
    }
}

cv::Mat to_cv(const Eigen::Matrix3d &R)
{
    cv::Mat mat;
    cv::eigen2cv(R, mat);
    return mat;
}

// 用真值造一组自洽的手眼数据:云台姿态任取,板位姿由 AX=BX 反推,标定结果应精确还原真值。
std::vector<tools::calibration::HandEyeSample> make_samples(const Eigen::Matrix3d &r_camera2gimbal, const Eigen::Vector3d &t_camera2gimbal)
{
    const Eigen::Matrix3d r_board2world = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d t_board2world(0.2, 0.4, 2.0);

    std::mt19937                           gen(42); // NOLINT(bugprone-random-generator-seed)
    std::uniform_real_distribution<double> angle(-M_PI, M_PI);

    std::vector<tools::calibration::HandEyeSample> samples;
    for (int i = 0; i < 20; ++i)
    {
        const Eigen::Matrix3d r_gimbal2world =
            (Eigen::AngleAxisd(angle(gen), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(angle(gen), Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(angle(gen), Eigen::Vector3d::UnitZ()))
                .toRotationMatrix();
        const Eigen::Matrix3d r_board2camera = r_camera2gimbal.transpose() * r_gimbal2world.transpose() * r_board2world;
        const Eigen::Vector3d t_board2camera = r_camera2gimbal.transpose() * (r_gimbal2world.transpose() * t_board2world - t_camera2gimbal);

        tools::calibration::HandEyeSample sample;
        sample.r_gimbal2world = to_cv(r_gimbal2world);
        sample.t_gimbal2world = cv::Mat::zeros(3, 1, CV_64F);
        cv::Rodrigues(to_cv(r_board2camera), sample.rvec);
        cv::eigen2cv(t_board2camera, sample.tvec);
        samples.push_back(sample);
    }
    return samples;
}
} // namespace

int main()
try
{
    // 真值:相机相对云台转了几度,并有一个小平移
    const Eigen::Matrix3d r_camera2gimbal =
        (Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-3.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()))
            .toRotationMatrix();
    const Eigen::Vector3d t_camera2gimbal(0.01, -0.02, 0.03);

    const auto samples = make_samples(r_camera2gimbal, t_camera2gimbal);
    const auto result  = tools::calibration::calibrate_hand_eye(samples);

    Eigen::Matrix3d r_estimate;
    cv::cv2eigen(result.r_camera2gimbal, r_estimate);
    Eigen::Vector3d t_estimate;
    cv::cv2eigen(result.t_camera2gimbal, t_estimate);

    const double angle_deg    = Eigen::AngleAxisd(r_camera2gimbal.transpose() * r_estimate).angle() * 180.0 / M_PI;
    const double offset_meter = (t_estimate - t_camera2gimbal).norm();

    check(angle_deg < 0.05, "rotation recovered, deviation " + std::to_string(angle_deg) + " deg");
    check(offset_meter < 1.0e-3, "translation recovered, offset " + std::to_string(offset_meter) + " m");
    check(result.consistency_deg < 0.05, "consistency " + std::to_string(result.consistency_deg) + " deg");

    // 样本不足要报错
    bool threw = false;
    try
    {
        static_cast<void>(tools::calibration::calibrate_hand_eye({samples[0], samples[1]}));
    }
    catch (const std::invalid_argument &)
    {
        threw = true;
    }
    check(threw, "throws with less than 3 samples");

    std::cout << (failures == 0 ? "calibration test passed" : "calibration test failed") << '\n';
    return failures == 0 ? 0 : 1;
}
catch (const std::exception &error)
{
    std::cerr << "FAIL: exception " << error.what() << '\n';
    return 1;
}
