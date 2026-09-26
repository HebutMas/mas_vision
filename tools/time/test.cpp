#include "tools/time/time.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <string>

namespace
{
int failures = 0;

void check(bool ok, const std::string & what)
{
  if (!ok)
  {
    std::cerr << "FAIL: " << what << "\n";
    ++failures;
  }
}
} // namespace

int main()
try
{
  // 统一起点:多次调用必须返回同一个值。
  check(tools::time::base() == tools::time::base(), "base is stable");

  // 单调:后取的时刻不早于先取的。
  const auto first = tools::time::now();
  const auto second = tools::time::now();
  check(second >= first, "now is monotonic");

  // 相对统一起点的时长非负,且随时刻推进不减。
  check(tools::time::since_base(first).count() >= 0, "since_base is non-negative");
  check(tools::time::since_base(second) >= tools::time::since_base(first),
        "since_base grows with time");

  // 与直接相减一致:证明所有调用共享同一基准。
  const auto delta = std::chrono::duration_cast<std::chrono::nanoseconds>(second - first);
  check(tools::time::since_base(second) - tools::time::since_base(first) == delta,
        "since_base shares one base");

  std::cout << (failures == 0 ? "time test passed\n" : "time test failed\n");
  return failures == 0 ? 0 : 1;
}
catch (const std::exception & error)
{
  std::cerr << "time_test error: " << error.what() << "\n";
  return 1;
}
