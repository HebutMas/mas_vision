#pragma once

#include <Eigen/Geometry>

#include <cstdint>

namespace hardware::serialport
{

struct ReceivePacket
{
    std::uint8_t mode;           // 自瞄模式。
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
