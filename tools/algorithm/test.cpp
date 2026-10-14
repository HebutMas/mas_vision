#include "tools/algorithm/crc16.hpp"
#include "tools/algorithm/extended_kalman_filter.hpp"
#include "tools/algorithm/math_tools.hpp"
#include "tools/algorithm/quaternion_buffer.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>

namespace
{
int failures = 0;

void check(bool ok, const char *what)
{
    if (!ok)
    {
        std::cerr << "FAIL: " << what << "\n";
        ++failures;
    }
}

bool near(float a, float b, float eps = 1e-4F) { return std::fabs(a - b) <= eps; }
} // namespace

int main()
{
    // CRC-16/MCRF4XX 标准测试向量(poly 0x8408, init 0xFFFF, 反射)。
    const auto *data = reinterpret_cast<const std::uint8_t *>("123456789");
    check(tools::crc16::checksum(data, 9) == 0x6F91, "crc16 known vector");
    check(tools::crc16::checksum(nullptr, 0) == 0xFFFF, "crc16 empty init");

    // 四元数缓冲:空 -> 单位;0 → 90°(绕 Z)的中点应为 45°;越界取端点。
    tools::TimedQuaternionBuffer<8> buffer;
    check(near(buffer.at(tools::time::now()).w(), 1.0F), "empty -> identity");

    const auto  t0 = tools::time::base();
    const float h  = std::sqrt(0.5F);
    buffer.push(t0, Eigen::Quaternionf::Identity());
    buffer.push(t0 + std::chrono::milliseconds(10), Eigen::Quaternionf(h, 0.0F, 0.0F, h)); // 90° about Z

    const float eighth = 0.3926991F; // pi/8 = 22.5°
    const auto  mid    = buffer.at(t0 + std::chrono::milliseconds(5));
    check(near(mid.w(), std::cos(eighth)) && near(mid.z(), std::sin(eighth)), "slerp midpoint ~45deg");

    const auto before = buffer.at(t0 - std::chrono::seconds(1));
    check(near(before.w(), 1.0F) && near(before.z(), 0.0F), "clamp oldest");
    const auto after = buffer.at(t0 + std::chrono::seconds(1));
    check(near(after.w(), h) && near(after.z(), h), "clamp newest");

    // clear 后回到单位四元数
    buffer.clear();
    check(near(buffer.at(t0).w(), 1.0F), "clear -> identity");

    // ===== P0: math_tools =====
    check(std::fabs(tools::limit_rad((2 * M_PI) + 0.3) - 0.3) < 1e-12, "limit_rad wrap +");
    check(std::fabs(tools::limit_rad((-2 * M_PI) - 0.5) + 0.5) < 1e-12, "limit_rad wrap -");

    const Eigen::Vector3d ypd(1.0, 0.5, 2.0); // yaw, pitch, dist
    check((tools::xyz2ypd(tools::ypd2xyz(ypd)) - ypd).cwiseAbs().maxCoeff() < 1e-9, "ypd <-> xyz roundtrip");

    { // 雅可比 = 数值差分
        auto                  f = [](const Eigen::Vector3d &v) { return tools::xyz2ypd(v); };
        const Eigen::Vector3d p(2.0, 1.0, 0.3);
        const Eigen::MatrixXd jac  = tools::xyz2ypd_jacobian(p);
        const double          step = 1e-6;
        for (int i = 0; i < 3; ++i)
        {
            Eigen::Vector3d dp            = Eigen::Vector3d::Zero();
            dp[i]                         = step;
            const Eigen::Vector3d numeric = (f(p + dp) - f(p - dp)) / (2 * step);
            check((numeric - jac.col(i)).cwiseAbs().maxCoeff() < 1e-8, "xyz2ypd_jacobian vs numeric");
        }
    }

    // ===== P0: EKF(匀速点 + 角度回卷) =====
    { // 状态 [x vx y vy]，位置观测收敛于真值
        Eigen::VectorXd x0(4);
        x0 << 0.1, 0.0, -0.05, 0.0;
        const Eigen::MatrixXd       p0 = Eigen::MatrixXd::Identity(4, 4) * 10.0;
        tools::ExtendedKalmanFilter ekf(x0, p0);

        const double    dt          = 0.05;
        Eigen::MatrixXd f_mat       = Eigen::Matrix4d::Identity();
        f_mat(0, 1)                 = dt;
        f_mat(2, 3)                 = dt;
        const Eigen::MatrixXd q_mat = Eigen::MatrixXd::Identity(4, 4) * 1e-3;
        Eigen::MatrixXd       h_mat(2, 4);
        h_mat << 1, 0, 0, 0, 0, 0, 1, 0;
        const Eigen::MatrixXd r_mat = Eigen::MatrixXd::Identity(2, 2) * 1e-2;

        double t = 0.0;
        for (int i = 0; i < 60; ++i)
        {
            t += dt;
            ekf.predict(f_mat, q_mat);
            const Eigen::Vector2d truth(0.5 * t, -0.2 * t); // 真实匀速 (0.5, -0.2)
            ekf.update(truth, h_mat, r_mat);
        }
        const double pos_err = std::hypot(ekf.x[0] - (0.5 * t), ekf.x[2] + (0.2 * t));
        const bool   vel_ok  = std::fabs(ekf.x[1] - 0.5) < 0.05 && std::fabs(ekf.x[3] + 0.2) < 0.05;
        check(pos_err < 0.05 && vel_ok, "ekf cv convergence");
        check(ekf.data["nis"] > 0.0 && !ekf.recent_nis_failures.empty(), "ekf nis bookkeeping");
    }

    { // χ² 标定:真值噪声与 Q/R 同源时,平稳 NIS 均值应收敛到观测维度 m=2。
        // 固定种子保证可复现;若用后验残差配后验 P 算 NIS,均值会被压到 ~0.1,本用例即失败。
        Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
        const Eigen::MatrixXd p0 = Eigen::MatrixXd::Identity(4, 4);
        tools::ExtendedKalmanFilter ekf(x0, p0);

        const double dt = 0.05, sig_a = 2.0, sig_r = 0.1;
        Eigen::MatrixXd f_mat = Eigen::Matrix4d::Identity();
        f_mat(0, 1) = f_mat(2, 3) = dt;
        // 分段常值加速度模型的过程噪声 Q = σ_a² · [[dt⁴/4, dt³/2], [dt³/2, dt²]]
        const double q_p = dt * dt * dt * dt / 4.0 * sig_a * sig_a;
        const double q_m = dt * dt * dt / 2.0 * sig_a * sig_a;
        const double q_v = dt * dt * sig_a * sig_a;
        Eigen::MatrixXd q_mat = Eigen::MatrixXd::Zero(4, 4);
        q_mat(0, 0) = q_mat(2, 2) = q_p;
        q_mat(0, 1) = q_mat(1, 0) = q_mat(2, 3) = q_mat(3, 2) = q_m;
        q_mat(1, 1) = q_mat(3, 3) = q_v;
        Eigen::MatrixXd h_mat(2, 4);
        h_mat << 1, 0, 0, 0, 0, 0, 1, 0;
        const Eigen::MatrixXd r_mat = Eigen::MatrixXd::Identity(2, 2) * sig_r * sig_r;

        std::mt19937           rng(20261013U);
        std::normal_distribution<double> acc(0.0, sig_a), meas(0.0, sig_r);
        Eigen::Vector4d        truth = Eigen::Vector4d::Zero();
        double                 sum   = 0.0;
        int                    n     = 0;
        for (int i = 0; i < 3000; ++i)
        {
            const double ax = acc(rng), ay = acc(rng);
            truth[0] += truth[1] * dt + ax * dt * dt / 2.0;
            truth[1] += ax * dt;
            truth[2] += truth[3] * dt + ay * dt * dt / 2.0;
            truth[3] += ay * dt;
            ekf.predict(f_mat, q_mat);
            const Eigen::Vector2d z(truth[0] + meas(rng), truth[2] + meas(rng));
            ekf.update(z, h_mat, r_mat);
            if (i >= 300)
            {
                sum += ekf.last_nis;
                ++n;
            }
        }
        const double mean_nis = sum / static_cast<double>(n);
        std::cerr << "nis mean = " << mean_nis << " (期望 ~2)\n";
        check(mean_nis > 1.4 && mean_nis < 2.6, "ekf nis chi2 calibration (mean ~ m=2)");
    }

    { // 一维角度状态 θ，已知 ω=0.4，跨多圈回卷仍跟住
        Eigen::VectorXd x0(1);
        x0 << 2.9;
        const Eigen::MatrixXd p0    = Eigen::MatrixXd::Identity(1, 1) * 1.0;
        auto                  x_add = [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) {
            Eigen::VectorXd r = a;
            r[0] += tools::limit_rad(b[0]);
            return r;
        };
        auto z_sub = [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) {
            Eigen::VectorXd r = a;
            r[0] -= b[0];
            r[0] = tools::limit_rad(r[0]);
            return r;
        };
        tools::ExtendedKalmanFilter ekf(x0, p0, x_add);

        const Eigen::MatrixXd f_mat = Eigen::MatrixXd::Ones(1, 1);
        const Eigen::MatrixXd q_mat = Eigen::MatrixXd::Ones(1, 1) * 1e-6;
        const Eigen::MatrixXd h_mat = Eigen::MatrixXd::Ones(1, 1);
        const Eigen::MatrixXd r_mat = Eigen::MatrixXd::Ones(1, 1) * 1e-2;
        auto                  f     = [](const Eigen::VectorXd &x) {
            Eigen::VectorXd r = x;
            r[0]              = tools::limit_rad(r[0] + 0.4);
            return r;
        };

        double truth = 2.9;
        for (int i = 0; i < 40; ++i)
        {
            ekf.predict(f_mat, q_mat, f);
            truth = tools::limit_rad(truth + 0.4);
            Eigen::VectorXd z(1);
            z << truth;
            ekf.update(z, h_mat, r_mat, z_sub);
        }
        const double err = std::fabs(tools::limit_rad(ekf.x[0] - truth));
        check(err < 0.2, "ekf angle wrap tracking");
    }

    if (failures != 0)
    {
        std::cerr << "algorithm test failed: " << failures << "\n";
        return 1;
    }
    std::cout << "algorithm test passed\n";
    return 0;
}
