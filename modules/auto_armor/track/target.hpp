#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "modules/auto_armor/armor.hpp"
#include "tools/algorithm/extended_kalman_filter.hpp"

// 模型层:单台车的ekf整车 11 维
//   状态 x = [xc, vxc, yc, vyc, zc, vzc, yaw, w, r, l, h]
//   几何   第 id 块板位于 center - r_i·(cosθ, sinθ),高度 center.z + h_i,
//          θ = yaw + id·2π/N,r_i = r (+l,奇数板),h_i = h (奇数板)
//   观测 z = [方位 φ, 俯仰 p, 距离 d, 板朝向 ψ],ψ = θ;H 为解析雅可比。
namespace rm::armor::track
{
// 整车 EKF 参数
struct ModelParams
{
    int    armor_num{4};
    double radius{0.26};  // 首帧 r 初值(收敛后由 EKF 估计)
    double q_rlh{1.0e-4}; // r/l/h 过程噪声系数

    // 白噪声加速度分段模型:v1 平移、v2 角加速度方差
    double v1{100.0}, v2{400.0};
    double slow_w{2.0}, slow_v1{10.0}, slow_v2{0.5};     // |w| < slow_w
    double fast_w{8.0}, fast_v1{500.0}, fast_v2{1000.0}; // |w| >= fast_w

    // 观测噪声(对角):[yaw, pitch, dist, armor_yaw]
    double r_yaw{4.0e-3}, r_pitch{4.0e-3}, r_dist{9.0e-2}, r_armor_yaw{9.0e-2};

    // P0 对角(顺序 x,vx,y,vy,z,vz,yaw,w,r,l,h)
    Eigen::Matrix<double, 11, 1> p0{(Eigen::Matrix<double, 11, 1>() << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1).finished()};

    bool   outpost_w_clamp{false}; // 收敛后 |w| 钳到 outpost_w
    double outpost_w{2.51};        // 前哨站标准转速
};

// track 输出
struct TargetState
{
    bool            tracking{false};                          // tracking 或 temp_lost
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};          // EKF 估计的整车中心位置
    Eigen::Vector3d vel{Eigen::Vector3d::Zero()};             // EKF 估计的整车速度
    double          yaw{0.0}, w{0.0}, r{0.0}, l{0.0}, h{0.0}; // EKF 估计的整车几何
    int             armor_num{0};                             // 车型的板块数
    int             last_id{0};                               // 最近匹配的板 id
    bool            jumped{false};                            // 是否跳板
    Kind            kind{Kind::infantry1};                    // 最近匹配观测的兵种
    Color           color{Color::blue};                       // 最近匹配观测的敌我色
    float           confidence{0.0F};                         // 最近匹配观测的置信度

    [[nodiscard]] Eigen::Vector3d armor_xyz(int id) const;
    [[nodiscard]] double          armor_yaw(int id) const;
};

// 一台车的 11 维状态 EKF 目标
class Target
{
  public:
    Target(const Armor3d &obs, ModelParams params);

    [[nodiscard]] Kind kind() const { return kind_; }
    [[nodiscard]] int  last_id() const { return last_id_; }

    // 预测滤波器状态,按 dt 分段,根据 w 选 v1/v2。
    void predict(double dt);

    struct Match
    {
        int    id;    // 命中的板 id
        double maha2; // 马氏距离平方
        double e_yaw; // 该板朝向残差(跳板判定用)
    };

    // 与该观测关联的最佳板 id,马氏距离超门限返回 nullopt。
    [[nodiscard]] std::optional<Match> match(const Armor3d &obs, double maha_gate) const;

    // 用该观测修正滤波器
    void update(const Armor3d &obs, int id);

    // 自恢复:中心重锚到观测板,半径/速度重置,但保留转速 w。
    void recover(const Armor3d &obs, int id);

    // 健康检查，几何发散或 NIS 失败比例超限。
    [[nodiscard]] bool unhealthy(double nis_reset_ratio) const;

    [[nodiscard]] TargetState state(bool jumped) const;

  private:
    [[nodiscard]] Eigen::MatrixXd jacobian(int id) const;
    [[nodiscard]] Eigen::Matrix4d obs_noise(const Armor3d &obs) const;

    tools::ExtendedKalmanFilter ekf_;
    ModelParams                 params_;
    Kind                        kind_;
    Color                       color_{Color::blue};
    float                       confidence_{0.0F};
    int                         last_id_{0};

    enum Tier : std::uint8_t
    {
        slow,
        normal,
        fast,
    };
    Tier tier_{normal};
};

// 纯外推,不动滤波器状态。
[[nodiscard]] TargetState                  predict_at(const TargetState &s, double t_future);
[[nodiscard]] std::vector<Eigen::Vector3d> armor_positions_at(const TargetState &s, double t_future);

} // namespace rm::armor::track
