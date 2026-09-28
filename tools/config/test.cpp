#include "tools/config/config.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace
{
int failures = 0;

void check(bool ok, const std::string &what)
{
    if (!ok)
    {
        std::cerr << "FAIL: " << what << "\n";
        ++failures;
    }
}

constexpr const char *YAML_TEXT = R"(camera:
  type: opencv
  exposure:
    time: 250
width: 640
ratio: 0.75
empty: null
)";

// 返回临时 yaml 文件路径,进程退出前由调用方删除。
std::filesystem::path write_fixture()
{
    const auto    path = std::filesystem::temp_directory_path() / "rm_vision_config_test.yaml";
    std::ofstream out(path);
    out << YAML_TEXT;
    return path;
}
} // namespace

int main()
try
{
    const auto                  path = write_fixture();
    const tools::config::Config config(path.string());

    check(config.require<std::string>("camera.type") == "opencv", "read nested value");
    check(config.require<int>("camera.exposure.time") == 250, "read deep nested value");
    check(config.require<int>("width") == 640, "read top-level value");
    check(config.require<double>("ratio") == 0.75, "read double value");

    // 必填项:缺失 / null / 类型不符都抛错。
    auto throws = [](auto &&fn) {
        try
        {
            fn();
        }
        catch (const std::exception &)
        {
            return true;
        }
        return false;
    };
    check(throws([&] { (void)config.require<std::string>("missing"); }), "missing throws");
    check(throws([&] { (void)config.require<std::string>("empty"); }), "null throws");
    check(throws([&] { (void)config.require<std::string>("serialport.device"); }), "missing section throws");
    check(throws([&] { (void)config.require<int>("camera.type"); }), "type mismatch throws");

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
catch (const std::exception &error)
{
    std::cerr << "config_test error: " << error.what() << "\n";
    return 1;
}
