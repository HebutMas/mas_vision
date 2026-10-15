#include "math_tools.hpp"

#include <cmath>

namespace tools
{
double limit_rad(double angle)
{
    constexpr double PI = M_PI;
    while (angle > PI) angle -= 2 * PI;
    while (angle <= -PI) angle += 2 * PI;
    return angle;
}

Eigen::Vector3d eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic)
{
    if (!extrinsic) std::swap(axis0, axis2);

    const auto i         = axis0;
    const auto j         = axis1;
    auto       k         = axis2;
    const auto is_proper = (i == k);
    if (is_proper) k = 3 - i - j;
    const auto sign = ((i - j) * (j - k) * (k - i)) / 2;

    double                a    = 0.0;
    double                b    = 0.0;
    double                c    = 0.0;
    double                d    = 0.0;
    const Eigen::Vector4d xyzw = q.coeffs();
    if (is_proper)
    {
        a = xyzw[3];
        b = xyzw[i];
        c = xyzw[j];
        d = xyzw[k] * sign;
    }
    else
    {
        a = xyzw[3] - xyzw[j];
        b = xyzw[i] + (xyzw[k] * sign);
        c = xyzw[j] + xyzw[3];
        d = (xyzw[k] * sign) - xyzw[i];
    }

    Eigen::Vector3d eulers;
    const auto      n2 = (a * a) + (b * b) + (c * c) + (d * d);
    eulers[1]          = std::acos(((2 * ((a * a) + (b * b))) / n2) - 1);

    const auto half_sum  = std::atan2(b, a);
    const auto half_diff = std::atan2(-d, c);

    const auto eps   = 1e-7;
    const auto safe1 = std::abs(eulers[1]) >= eps;
    const auto safe2 = std::abs(eulers[1] - M_PI) >= eps;
    const auto safe  = safe1 && safe2;
    if (safe)
    {
        eulers[0] = half_sum + half_diff;
        eulers[2] = half_sum - half_diff;
    }
    else
    {
        if (!extrinsic)
        {
            eulers[0] = 0;
            if (!safe1) eulers[2] = 2 * half_sum;
            if (!safe2) eulers[2] = -2 * half_diff;
        }
        else
        {
            eulers[2] = 0;
            if (!safe1) eulers[0] = 2 * half_sum;
            if (!safe2) eulers[0] = 2 * half_diff;
        }
    }

    for (int i = 0; i < 3; i++) eulers[i] = limit_rad(eulers[i]);

    if (!is_proper)
    {
        eulers[2] *= sign;
        eulers[1] -= M_PI / 2;
    }

    if (!extrinsic) std::swap(eulers[0], eulers[2]);

    return eulers;
}

Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic)
{
    const Eigen::Quaterniond q(R);
    return eulers(q, axis0, axis1, axis2, extrinsic);
}

Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d &ypr)
{
    const double roll      = ypr[2];
    const double pitch     = ypr[1];
    const double yaw       = ypr[0];
    const double cos_yaw   = cos(yaw);
    const double sin_yaw   = sin(yaw);
    const double cos_pitch = cos(pitch);
    const double sin_pitch = sin(pitch);
    const double cos_roll  = cos(roll);
    const double sin_roll  = sin(roll);
    // clang-format off
    Eigen::Matrix3d rot{
      {cos_yaw * cos_pitch, (cos_yaw * sin_pitch * sin_roll) - (sin_yaw * cos_roll), (cos_yaw * sin_pitch * cos_roll) + (sin_yaw * sin_roll)},
      {sin_yaw * cos_pitch, (sin_yaw * sin_pitch * sin_roll) + (cos_yaw * cos_roll), (sin_yaw * sin_pitch * cos_roll) - (cos_yaw * sin_roll)},
      {         -sin_pitch,                                   cos_pitch * sin_roll,                                 cos_pitch * cos_roll}
    };
    // clang-format on
    return rot;
}

Eigen::Vector3d xyz2ypd(const Eigen::Vector3d &xyz)
{
    const auto x        = xyz[0];
    const auto y        = xyz[1];
    const auto z        = xyz[2];
    const auto yaw      = std::atan2(y, x);
    const auto pitch    = std::atan2(z, std::sqrt((x * x) + (y * y)));
    const auto distance = std::sqrt((x * x) + (y * y) + (z * z));
    return {yaw, pitch, distance};
}

Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d &xyz)
{
    const auto x   = xyz[0];
    const auto y   = xyz[1];
    const auto z   = xyz[2];
    const auto rxy = (x * x) + (y * y);

    const auto dyaw_dx = -y / rxy;
    const auto dyaw_dy = x / rxy;
    const auto dyaw_dz = 0.0;

    const auto dpitch_dx = -((x * z) / ((((z * z) / rxy) + 1) * (rxy * std::sqrt(rxy))));
    const auto dpitch_dy = -((y * z) / ((((z * z) / rxy) + 1) * (rxy * std::sqrt(rxy))));
    const auto dpitch_dz = 1 / ((((z * z) / rxy) + 1) * std::sqrt(rxy));

    const auto ddistance_dx = x / std::sqrt((x * x) + (y * y) + (z * z));
    const auto ddistance_dy = y / std::sqrt((x * x) + (y * y) + (z * z));
    const auto ddistance_dz = z / std::sqrt((x * x) + (y * y) + (z * z));

    // clang-format off
  Eigen::MatrixXd jac{
    {dyaw_dx, dyaw_dy, dyaw_dz},
    {dpitch_dx, dpitch_dy, dpitch_dz},
    {ddistance_dx, ddistance_dy, ddistance_dz}
  };
    // clang-format on

    return jac;
}

Eigen::Vector3d ypd2xyz(const Eigen::Vector3d &ypd)
{
    const auto yaw   = ypd[0];
    const auto pitch = ypd[1];
    const auto dist  = ypd[2];
    const auto x     = dist * cos(pitch) * cos(yaw);
    const auto y     = dist * cos(pitch) * sin(yaw);
    const auto z     = dist * sin(pitch);
    return {x, y, z};
}

Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d &ypd)
{
    const auto   yaw       = ypd[0];
    const auto   pitch     = ypd[1];
    const auto   dist      = ypd[2];
    const double cos_yaw   = cos(yaw);
    const double sin_yaw   = sin(yaw);
    const double cos_pitch = cos(pitch);
    const double sin_pitch = sin(pitch);

    const auto dx_dyaw = dist * cos_pitch * -sin_yaw;
    const auto dy_dyaw = dist * cos_pitch * cos_yaw;
    const auto dz_dyaw = 0.0;

    const auto dx_dpitch = dist * -sin_pitch * cos_yaw;
    const auto dy_dpitch = dist * -sin_pitch * sin_yaw;
    const auto dz_dpitch = dist * cos_pitch;

    const auto dx_ddist = cos_pitch * cos_yaw;
    const auto dy_ddist = cos_pitch * sin_yaw;
    const auto dz_ddist = sin_pitch;

    // clang-format off
  Eigen::MatrixXd jac{
    {dx_dyaw, dx_dpitch, dx_ddist},
    {dy_dyaw, dy_dpitch, dy_ddist},
    {dz_dyaw, dz_dpitch, dz_ddist}
  };
    // clang-format on

    return jac;
}

double delta_time(const std::chrono::steady_clock::time_point &a, const std::chrono::steady_clock::time_point &b)
{
    const std::chrono::duration<double> c = a - b;
    return c.count();
}

Eigen::Matrix3d gimbal2world(const Eigen::Quaterniond &q_imu, const Eigen::Matrix3d &r_gimbal2imubody)
{
    const Eigen::Matrix3d &a = r_gimbal2imubody;
    return a.transpose() * q_imu.normalized().toRotationMatrix() * a;
}

} // namespace tools
