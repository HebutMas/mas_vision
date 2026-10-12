#include "tools/algorithm/crc16.hpp"
#include "tools/algorithm/extended_kalman_filter.hpp"
#include "tools/algorithm/math_tools.hpp"
#include "tools/algorithm/quaternion_buffer.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>

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
