#include "modules/auto_armor/track/target.hpp"

#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

#include "tools/algorithm/math_tools.hpp"

namespace rm::armor::track
{

namespace
{
// 第 id 块板的几何:朝向 θ、等效半径 r_i、高度偏移 h_i。
struct BoardFrame
{
    double theta;
    double r_i;
    double h_i;
};

BoardFrame board_frame(double yaw, double r, double l, double h, int id, int n)
{
    const bool odd = ((id % 2) != 0);
    return {yaw + (id * 2 * M_PI / n), r + (odd ? l : 0.0), odd ? h : 0.0};
}

Eigen::Vector3d board_position(const Eigen::Vector3d &center, const BoardFrame &frame)
{
    return {center.x() - (frame.r_i * std::cos(frame.theta)), center.y() - (frame.r_i * std::sin(frame.theta)), center.z() + frame.h_i};
}

BoardFrame      frame_from(const Eigen::VectorXd &x, int id, int n) { return board_frame(x[6], x[8], x[9], x[10], id, n); }
Eigen::Vector3d position_from(const Eigen::VectorXd &x, int id, int n) { return board_position({x[0], x[2], x[4]}, frame_from(x, id, n)); }

// EKF 状态 -> 观测量 [方位 φ, 俯仰 p, 距离 d, 板朝向 ψ]。
Eigen::Vector4d armor_observe(const Eigen::VectorXd &x, int id, int n)
{
    const BoardFrame      frame = frame_from(x, id, n);
    const Eigen::Vector3d ypd   = tools::xyz2ypd(board_position({x[0], x[2], x[4]}, frame));
    return {ypd[0], ypd[1], ypd[2], tools::limit_rad(frame.theta)};
}

// 观测 -> 观测量 [φ, p, d, ψ]。
Eigen::Vector4d obs_vector(const Armor3d &obs)
{
    const Eigen::Vector3d ypd = tools::xyz2ypd(obs.xyz);
    return {ypd[0], ypd[1], ypd[2], tools::limit_rad(obs.yaw)};
}
} // namespace

Target::Target(const Armor3d &obs, ModelParams params) : params_(std::move(params)), kind_(obs.kind), color_(obs.color), confidence_(obs.confidence)
{
    const double    r   = params_.radius;
    const double    yaw = tools::limit_rad(obs.yaw); // 首帧观测板按 id=0 处理
    const double    cx  = r * std::cos(yaw);
    const double    cy  = r * std::sin(yaw);
    Eigen::VectorXd x0(11);
    x0 << obs.xyz.x() + cx, 0.0, obs.xyz.y() + cy, 0.0, obs.xyz.z(), 0.0, yaw, 0.0, r, 0.0, 0.0;

    auto x_add = [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) {
        Eigen::VectorXd out = a + b;
        out[6]              = tools::limit_rad(out[6]);
        return out;
    };
    ekf_     = tools::ExtendedKalmanFilter(x0, params_.p0.asDiagonal(), x_add);
    last_id_ = 0;
}

void Target::predict(double dt)
{
    // 档位选择(|w| 三档,0.5 rad/s 滞回防抖)。
    const double w = std::fabs(ekf_.x[7]);
    switch (tier_)
    {
    case slow:
        if (w > params_.slow_w + 0.5) tier_ = normal;
        break;
    case normal:
        if (w < params_.slow_w - 0.5)
        {
            tier_ = slow;
        }
        else if (w >= params_.fast_w)
        {
            tier_ = fast;
        }
        break;
    case fast:
        if (w < params_.fast_w - 0.5) tier_ = normal;
        break;
    }
    double v1 = params_.v1;
    double v2 = params_.v2;
    if (tier_ == slow)
    {
        v1 = params_.slow_v1;
        v2 = params_.slow_v2;
    }
    else if (tier_ == fast)
    {
        v1 = params_.fast_v1;
        v2 = params_.fast_v2;
    }

    Eigen::MatrixXd mat_f = Eigen::MatrixXd::Identity(11, 11);
    mat_f(0, 1) = mat_f(2, 3) = mat_f(4, 5) = dt;
    mat_f(6, 7)                             = dt;

    const double    dt2       = dt * dt;
    const double    dt3       = dt2 * dt;
    const double    dt4       = dt3 * dt;
    Eigen::MatrixXd mat_q     = Eigen::MatrixXd::Zero(11, 11);
    const auto      add_block = [&](int p, int v, double var) {
        mat_q(p, p) = (dt4 / 4) * var;
        mat_q(p, v) = mat_q(v, p) = (dt3 / 2) * var;
        mat_q(v, v)               = dt2 * var;
    };
    add_block(0, 1, v1);
    add_block(2, 3, v1);
    add_block(4, 5, v1);
    add_block(6, 7, v2);
    mat_q(8, 8) = mat_q(9, 9) = mat_q(10, 10) = params_.q_rlh * dt;

    auto propagate = [&](const Eigen::VectorXd &x) {
        Eigen::VectorXd out = mat_f * x;
        out[6]              = tools::limit_rad(out[6]);
        return out;
    };
    ekf_.predict(mat_f, mat_q, propagate);

    if (params_.outpost_w_clamp && std::fabs(ekf_.x[7]) > 2.0) // 收敛后按方向钳恒转速
    {
        ekf_.x[7] = ekf_.x[7] > 0 ? params_.outpost_w : -params_.outpost_w;
    }
}

std::optional<Target::Match> Target::match(const Armor3d &obs, double maha_gate) const
{
    // id 关联:对每个候选板算完整新息(方位/俯仰/距离/板朝向)的马氏距离,取最小者。
    // 只用 yaw 关联时,矩形板 yaw 的 180° 二义性会让 slot j 与 j+2 互相抢 id(表现为 id 在 1↔3 间翻);
    // 位置项(方位 φ+距离 d)对二者恰好相反,可自动打破二义性。
    const Eigen::Vector4d z     = obs_vector(obs);
    const Eigen::Matrix4d mat_r = obs_noise(obs);

    const int n = params_.armor_num;

    int    best_id   = 0;
    double best_d2   = std::numeric_limits<double>::infinity();
    double best_eyaw = 0.0;
    for (int id = 0; id < n; ++id)
    {
        Eigen::Vector4d e           = z - armor_observe(ekf_.x, id, n);
        e[0]                        = tools::limit_rad(e[0]);
        e[3]                        = tools::limit_rad(e[3]);
        const Eigen::MatrixXd mat_h = jacobian(id);
        const Eigen::Matrix4d mat_s = mat_h * ekf_.P * mat_h.transpose() + mat_r;
        const double          d2    = e.transpose() * mat_s.inverse() * e;
        if (d2 < best_d2)
        {
            best_d2   = d2;
            best_id   = id;
            best_eyaw = e[3];
        }
    }

    if (best_d2 >= maha_gate) return std::nullopt;
    return Match{best_id, best_d2, best_eyaw};
}

void Target::update(const Armor3d &obs, int id)
{
    const Eigen::VectorXd z     = obs_vector(obs);
    const Eigen::MatrixXd mat_h = jacobian(id);
    const Eigen::Matrix4d mat_r = obs_noise(obs);

    const int n     = params_.armor_num;
    auto      h     = [id, n](const Eigen::VectorXd &x) -> Eigen::VectorXd { return armor_observe(x, id, n); };
    auto      z_sub = [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) {
        Eigen::VectorXd out = a - b;
        out[0]              = tools::limit_rad(out[0]);
        out[3]              = tools::limit_rad(out[3]);
        return out;
    };
    ekf_.update(z, mat_h, mat_r, h, z_sub);
    last_id_    = id;
    color_      = obs.color;
    confidence_ = obs.confidence;
}

void Target::recover(const Armor3d &obs, int id)
{
    // 几何重置:半径/中心按观测重锚,位置/速度清零,但保留(w)
    ekf_.x[8]  = params_.radius;
    ekf_.x[9]  = 0.0;
    ekf_.x[10] = 0.0;
    ekf_.x[0]  = obs.xyz.x() + (params_.radius * std::cos(obs.yaw));
    ekf_.x[2]  = obs.xyz.y() + (params_.radius * std::sin(obs.yaw));
    ekf_.x[4]  = obs.xyz.z();
    ekf_.x[1] = ekf_.x[3] = ekf_.x[5] = 0.0;
    ekf_.x[6]                         = tools::limit_rad(obs.yaw - (id * 2 * M_PI / params_.armor_num));
    ekf_.P                            = params_.p0.asDiagonal();
    last_id_                          = id;
    color_                            = obs.color;
    confidence_                       = obs.confidence;
}

bool Target::unhealthy(double nis_reset_ratio) const
{
    // 几何发散
    const double r  = ekf_.x[8];
    const double rl = ekf_.x[8] + ekf_.x[9];
    if (r <= 0.05 || r >= 0.7) return true;
    if (rl <= 0.07 || rl >= 0.9) return true;
    if (std::fabs(ekf_.x[7]) > 50.0) return true;

    // 滑窗 NIS 失败比例过高
    if (ekf_.recent_nis_failures.empty()) return false;
    const double sum = std::accumulate(ekf_.recent_nis_failures.begin(), ekf_.recent_nis_failures.end(), 0.0);
    return (sum / static_cast<double>(ekf_.recent_nis_failures.size())) > nis_reset_ratio;
}

TargetState Target::state(bool jumped) const
{
    TargetState s;
    s.tracking   = true;
    s.center     = Eigen::Vector3d(ekf_.x[0], ekf_.x[2], ekf_.x[4]);
    s.vel        = Eigen::Vector3d(ekf_.x[1], ekf_.x[3], ekf_.x[5]);
    s.yaw        = ekf_.x[6];
    s.w          = ekf_.x[7];
    s.r          = ekf_.x[8];
    s.l          = ekf_.x[9];
    s.h          = ekf_.x[10];
    s.armor_num  = params_.armor_num;
    s.last_id    = last_id_;
    s.jumped     = jumped;
    s.kind       = kind_;
    s.color      = color_;
    s.confidence = confidence_;
    return s;
}

Eigen::Vector3d TargetState::armor_xyz(int id) const { return board_position(center, board_frame(yaw, r, l, h, id, armor_num)); }

double TargetState::armor_yaw(int id) const { return tools::limit_rad(yaw + (id * 2 * M_PI / armor_num)); }

TargetState predict_at(const TargetState &s, double t_future)
{
    TargetState out = s;
    out.center      = s.center + (s.vel * t_future);
    out.yaw         = tools::limit_rad(s.yaw + (s.w * t_future));
    return out;
}

std::vector<Eigen::Vector3d> armor_positions_at(const TargetState &s, double t_future)
{
    const TargetState            p = predict_at(s, t_future);
    std::vector<Eigen::Vector3d> positions;
    positions.reserve(p.armor_num);
    for (int id = 0; id < p.armor_num; ++id) positions.push_back(p.armor_xyz(id));
    return positions;
}

Eigen::MatrixXd Target::jacobian(int id) const
{
    const int        n     = params_.armor_num;
    const BoardFrame frame = frame_from(ekf_.x, id, n);
    const bool       odd   = (id % 2) != 0;

    // ∂(xa,ya,za)/∂x (3×11)
    Eigen::MatrixXd mat_jxyz = Eigen::MatrixXd::Zero(3, 11);
    mat_jxyz(0, 0)           = 1;
    mat_jxyz(1, 2)           = 1;
    mat_jxyz(2, 4)           = 1;
    mat_jxyz(0, 6)           = frame.r_i * std::sin(frame.theta);
    mat_jxyz(1, 6)           = -(frame.r_i * std::cos(frame.theta));
    mat_jxyz(0, 8)           = -std::cos(frame.theta);
    mat_jxyz(1, 8)           = -std::sin(frame.theta);
    if (odd)
    {
        mat_jxyz(0, 9)  = -std::cos(frame.theta);
        mat_jxyz(1, 9)  = -std::sin(frame.theta);
        mat_jxyz(2, 10) = 1;
    }

    // ρ < 0.3m 时钳到 0.3 防除零
    Eigen::Vector3d p   = position_from(ekf_.x, id, n);
    const double    rho = std::hypot(p.x(), p.y());
    if (rho < 0.3 && rho > 1e-9)
    {
        p.x() *= 0.3 / rho;
        p.y() *= 0.3 / rho;
    }

    Eigen::MatrixXd mat_h = Eigen::MatrixXd::Zero(4, 11);
    mat_h.topRows(3)      = tools::xyz2ypd_jacobian(p) * mat_jxyz;
    mat_h(3, 6)           = 1; // ψ = yaw + id·2π/N
    return mat_h;
}

Eigen::Matrix4d Target::obs_noise(const Armor3d &obs) const
{
    const Eigen::Vector3d ypd  = tools::xyz2ypd(obs.xyz);
    const double          beta = std::fabs(tools::limit_rad(obs.yaw - ypd[0] - M_PI));
    const double          d    = ypd[2];

    Eigen::Matrix4d mat_r = Eigen::Matrix4d::Zero();
    mat_r(0, 0)           = params_.r_yaw;
    mat_r(1, 1)           = params_.r_pitch;
    mat_r(2, 2)           = params_.r_dist * (std::log(std::fabs(beta) + 1) + 1);
    mat_r(3, 3)           = params_.r_armor_yaw + (std::log(d + 1) / 200);
    if (tier_ == fast)
    {
        mat_r(0, 0) *= 0.5;
        mat_r(1, 1) *= 0.5;
        mat_r(2, 2) = 0.05 * (std::log(std::fabs(beta) + 1) + 1);
    }
    return mat_r;
}

} // namespace rm::armor::track
