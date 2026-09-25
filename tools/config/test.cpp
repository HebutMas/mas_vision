#include "tools/config/config.hpp"

#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
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

constexpr const char * YAML_TEXT = R"(camera.type: opencv
width: 640
ratio: 0.75
empty: null
)";

// 返回临时 yaml 文件路径,进程退出前由调用方删除。
std::filesystem::path write_fixture()
{
  const auto path = std::filesystem::temp_directory_path() / "rm_vision_config_test.yaml";
  std::ofstream out(path);
  out << YAML_TEXT;
  return path;
}
} // namespace

int main()
try
{
  const auto path = write_fixture();
  const tools::config::Config config(path.string());

  check(config.get<std::string>("camera.type", "none") == "opencv", "read string value");
  check(config.get<int>("width", 0) == 640, "read int value");
  check(config.get<double>("ratio", 0.0) == 0.75, "read double value");
  check(config.get<std::string>("empty", "fallback") == "fallback", "null falls back");
  check(config.get<std::string>("missing", "fallback") == "fallback", "missing falls back");

  bool threw = false;
  try
  {
    const tools::config::Config bad("/nonexistent/rm_vision_config.yaml");
    (void)bad;
  }
  catch (const std::exception &)
  {
    threw = true;
  }
  check(threw, "missing file throws");

  std::filesystem::remove(path);
  std::cout << (failures == 0 ? "config test passed\n" : "config test failed\n");
  return failures == 0 ? 0 : 1;
}
catch (const std::exception & error)
{
  std::cerr << "config_test error: " << error.what() << "\n";
  return 1;
}
