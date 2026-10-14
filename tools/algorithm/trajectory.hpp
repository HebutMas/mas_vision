#pragma once

#include <cmath>

// 弹道解:抛物线闭式(无空气阻力)
namespace tools
{
struct Trajectory
{
    bool   unsolvable{false};
    double fly_time{0.0}; // s
    double pitch{0.0};    // rad,抬头为正
};

// v0 初速(m/s),d 水平距离(m),h 竖直高度(m)。超出射程返回 unsolvable。
[[nodiscard]] inline Trajectory solve_trajectory(double v0, double d, double h)
{
    constexpr double GRAVITY = 9.7833;
    constexpr double EPS     = 1.0e-9;

    Trajectory traj;
    if ((d < EPS) || (v0 < EPS))
    {
        traj.unsolvable = true;
        return traj;
    }

    // tanθ 的一元二次方程:a·tan²θ + b·tanθ + c = 0
    const double a     = GRAVITY * d * d / (2.0 * v0 * v0);
    const double b     = -d;
    const double c     = a + h;
    const double delta = (b * b) - (4.0 * a * c);
    if (delta < 0.0)
    {
        traj.unsolvable = true;
        return traj;
    }

    // 两根取低伸弹道(倾角小、飞行时间短)
    traj.pitch    = std::atan((-b - std::sqrt(delta)) / (2.0 * a));
    traj.fly_time = d / (v0 * std::cos(traj.pitch));
    return traj;
}
} // namespace tools
