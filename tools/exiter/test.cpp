#include "tools/exiter/exiter.hpp"

#include <csignal>
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
  tools::install_exit_handler();
  check(!tools::should_exit(), "not exiting before any signal");

  // 已安装处理函数,raise 不会终止进程,只会置位退出标志。
  std::raise(SIGINT);
  check(tools::should_exit(), "SIGINT sets the exit flag");

  std::cout << (failures == 0 ? "exiter test passed\n" : "exiter test failed\n");
  return failures == 0 ? 0 : 1;
}
catch (const std::exception & error)
{
  std::cerr << "exiter_test error: " << error.what() << "\n";
  return 1;
}
