#pragma once

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <map>

namespace tools
{
class ExtendedKalmanFilter
{
  public:
    Eigen::VectorXd x;
    Eigen::MatrixXd P;

    ExtendedKalmanFilter() = default;

    ExtendedKalmanFilter(
        Eigen::VectorXd x0, Eigen::MatrixXd p0,
        std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add = [](const Eigen::VectorXd &a,
                                                                                                    const Eigen::VectorXd &b) { return a + b; });

    Eigen::VectorXd predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q);

    Eigen::VectorXd predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q, const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> &f);

    Eigen::VectorXd update(
        const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R,
        const std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> &z_subtract =
            [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) { return a - b; });

    Eigen::VectorXd update(
        const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R,
        const std::function<Eigen::VectorXd(const Eigen::VectorXd &)>                          &h,
        const std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> &z_subtract =
            [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) { return a - b; });

    std::map<std::string, double> data; // 卡方检验数据
    std::deque<int>               recent_nis_failures{0};
    size_t                        window_size = 100;
    // NIS 失败阈值(χ²(4) 95% 分位 ≈ 9.488)。sp25 原文误用 0.711,此处修正。
    double nis_threshold = 9.5;
    double last_nis;

  private:
    Eigen::MatrixXd                                                                  identity_;
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add_;

    int nees_count_  = 0;
    int nis_count_   = 0;
    int total_count_ = 0;
};

} // namespace tools
