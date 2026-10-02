#pragma once

#include <Eigen/Geometry>

#include <cstdint>

namespace hardware::serialport
{

// 视觉模式:下位机通过串口下发,决定当前识别任务。
enum VisionMode : std::uint8_t
{
    auto_aim_red    = 0,
    auto_aim_blue   = 1,
    small_rune_red  = 2,
    small_rune_blue = 3,
    big_rune_red    = 4,
    big_rune_blue   = 5,
};

// 是否为能量机关(小符 / 大符)识别模式。
[[nodiscard]] constexpr bool is_rune_mode(std::uint8_t mode) { return mode >= small_rune_red && mode <= big_rune_blue; }

static_assert(!is_rune_mode(auto_aim_red));
static_assert(!is_rune_mode(auto_aim_blue));
static_assert(is_rune_mode(small_rune_red));
static_assert(is_rune_mode(big_rune_blue));

// 模式对应的阵营:偶数(0/2/4)为红方,奇数(1/3/5)为蓝方。
[[nodiscard]] constexpr bool is_red_mode(std::uint8_t mode) { return (mode % 2U) == 0U; }

static_assert(is_red_mode(auto_aim_red));
static_assert(!is_red_mode(auto_aim_blue));
static_assert(is_red_mode(small_rune_red));
static_assert(!is_red_mode(small_rune_blue));
static_assert(is_red_mode(big_rune_red));
static_assert(!is_red_mode(big_rune_blue));

struct ReceivePacket
{
    std::uint8_t mode;           // VisionMode。
    float        qw, qx, qy, qz; // 云台姿态四元数。

    // SerialPort 用它把每帧姿态送入 IMU 插值缓冲。
    [[nodiscard]] Eigen::Quaternionf quaternion() const { return {qw, qx, qy, qz}; }
} __attribute__((packed));

struct SendPacket
{
    float        target_yaw;   // 目标云台偏航角(弧度)。
    float        target_pitch; // 目标云台俯仰角(弧度)。
    std::uint8_t fire_advice;  // 0:不射击,1:射击。
} __attribute__((packed));

} // namespace hardware::serialport
