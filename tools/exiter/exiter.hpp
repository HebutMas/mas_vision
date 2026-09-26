#pragma once

#include <atomic>
#include <csignal>

namespace tools
{

// 进程退出标志:收到 SIGINT(Ctrl-C)/ SIGTERM 后置位。
inline std::atomic<bool> & exit_flag() noexcept
{
  static std::atomic<bool> flag{false};
  return flag;
}

// 信号处理函数。
inline void handle_exit_signal(int) noexcept
{
  exit_flag().store(true, std::memory_order_relaxed);
}

// 退出信号处理注册。应在启动阶段调用一次;
inline void install_exit_handler() noexcept
{
  std::signal(SIGINT, handle_exit_signal);
  std::signal(SIGTERM, handle_exit_signal);
}

// 是否已收到退出信号。主循环 / 各工作线程轮询此值以退出。
inline bool should_exit() noexcept
{
  return exit_flag().load(std::memory_order_relaxed);
}

} // namespace tools
