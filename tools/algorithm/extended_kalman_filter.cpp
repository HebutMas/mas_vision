#include "extended_kalman_filter.hpp"

#include <numeric>
#include <utility>

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(Eigen::VectorXd x0, Eigen::MatrixXd P0,
                                           std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
    : x(std::move(x0)), P(std::move(P0)), I(Eigen::MatrixXd::Identity(x.rows(), x.rows())), x_add(std::move(x_add))
{
    data["residual_yaw"]        = 0.0;
    data["residual_pitch"]      = 0.0;
    data["residual_distance"]   = 0.0;
    data["residual_angle"]      = 0.0;
    data["nis"]                 = 0.0;
    data["nees"]                = 0.0;
    data["nis_fail"]            = 0.0;
    data["nees_fail"]           = 0.0;
    data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q)
{
    return predict(F, Q, [&](const Eigen::VectorXd &x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q,
                                              const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> &f)
{
    P = F * P * F.transpose() + Q;
    x = f(x);
    return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R,
                                             const std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> &z_subtract)
{
    return update(z, H, R, [&](const Eigen::VectorXd &x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R,
                                             const std::function<Eigen::VectorXd(const Eigen::VectorXd &)>                          &h,
                                             const std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> &z_subtract)
{
    const Eigen::VectorXd x_prior     = x;
    const Eigen::MatrixXd kalman_gain = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // Stable Compution of the Posterior Covariance
    // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
    P = (I - kalman_gain * H) * P * (I - kalman_gain * H).transpose() + kalman_gain * R * kalman_gain.transpose();

    x = x_add(x, kalman_gain * z_subtract(z, h(x)));

    /// 卡方检验
    const Eigen::VectorXd residual       = z_subtract(z, h(x));
    const Eigen::MatrixXd innovation_cov = H * P * H.transpose() + R;
    const double          nis            = residual.transpose() * innovation_cov.inverse() * residual;
    const double          nees           = (x - x_prior).transpose() * P.inverse() * (x - x_prior);

    // 卡方检验阈值（自由度=4，取置信水平95%）
    constexpr double NIS_THRESHOLD  = 0.711;
    constexpr double NEES_THRESHOLD = 0.711;

    if (nis > NIS_THRESHOLD) nis_count_++, data["nis_fail"] = 1;
    if (nees > NEES_THRESHOLD) nees_count_++, data["nees_fail"] = 1;
    total_count_++;
    last_nis = nis;

    recent_nis_failures.push_back(nis > NIS_THRESHOLD ? 1 : 0);

    if (recent_nis_failures.size() > window_size)
    {
        recent_nis_failures.pop_front();
    }

    const int    recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
    const double recent_rate     = static_cast<double>(recent_failures) / recent_nis_failures.size();

    if (residual.size() > 0) data["residual_yaw"] = residual[0];
    if (residual.size() > 1) data["residual_pitch"] = residual[1];
    if (residual.size() > 2) data["residual_distance"] = residual[2];
    if (residual.size() > 3) data["residual_angle"] = residual[3];
    data["nis"]                 = nis;
    data["nees"]                = nees;
    data["recent_nis_failures"] = recent_rate;

    return x;
}

} // namespace tools
