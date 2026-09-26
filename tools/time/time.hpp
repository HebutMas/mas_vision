#pragma once

#include <chrono>

namespace tools::time
{

// 统一时间基准:所有硬件(相机、IMU、串口等)都基于同一个单调时钟打时间戳
using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

// 当前时刻。
inline TimePoint now() noexcept
{
  return Clock::now();
}

// 进程级统一起点:首次访问时记录,之后所有硬件共用,保证跨设备时长可直接相减比较。
inline TimePoint base() noexcept
{
  static const TimePoint origin = Clock::now();
  return origin;
}

// 相对统一起点的时长,用于按真实时间回放 / 跨传感器对齐。
inline std::chrono::nanoseconds since_base(TimePoint point) noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(point - base());
}

} // namespace tools::time
