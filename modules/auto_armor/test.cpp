#include "modules/auto_armor/debug/visualize.hpp"
#include "modules/auto_armor/detection/detector.hpp"
#include "tools/config/config.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace
{

int failures = 0;

[[nodiscard]] bool close(const cv::Point2f &lhs, const cv::Point2f &rhs, float tolerance = 1E-3F)
{
    return std::abs(lhs.x - rhs.x) <= tolerance && std::abs(lhs.y - rhs.y) <= tolerance;
}

void check(bool ok, const std::string &what)
{
    if (!ok)
    {
        std::cerr << "FAIL: " << what << "\n";
        ++failures;
    }
}

// 临时 YAML:只写检测器需要的键(模型 + 设备),其余走默认值。
[[nodiscard]] std::string test_config()
{
    const std::string path = (std::filesystem::temp_directory_path() / "rm_auto_armor_test.yaml").string();
    std::ofstream     out(path);
    out << "auto_aim:\n  detector:\n    model: " << RM_AUTO_ARMOR_MODELS_DIR << "/shenzhen-0526.onnx\n    device: CPU\n";
    return path;
}

// 图上叠 HUD:耗时 + 数量 + 端点来源 + 颜色图例。
cv::Mat annotate(const cv::Mat &image, const rm::armor::Detector::Result &result, double latency_ms)
{
    cv::Mat vis = image.clone();
    rm::armor::draw(vis, result);

    const double     scale     = std::max(0.8, image.rows / 720.0);
    const int        thickness = std::max(1, image.rows / 540);
    const cv::Scalar yellow{0, 255, 255};
    const cv::Scalar green{0, 255, 0};

    const std::array<std::string, 2> lines{
        cv::format("armor %d | lightbar %d | %.2f ms", static_cast<int>(result.armors.size()), static_cast<int>(result.lightbars.size()), latency_ms),
        "box yellow | corners green ",
    };
    const std::array<double, 2> font_scales{0.8 * scale, 0.4 * scale};

    // 一层半透明黑底
    int widest = 0;
    int height = 4;
    for (std::size_t index = 0; index < lines.size(); ++index)
    {
        int            baseline = 0;
        const cv::Size size     = cv::getTextSize(lines[index], cv::FONT_HERSHEY_SIMPLEX, font_scales[index], thickness, &baseline);
        widest                  = std::max(widest, size.width);
        height += size.height + 6;
    }
    cv::Mat overlay = vis.clone();
    cv::rectangle(overlay, {0, 0, std::min(widest + 12, vis.cols), std::min(height, vis.rows)}, {0, 0, 0}, cv::FILLED);
    cv::addWeighted(overlay, 0.45, vis, 0.55, 0.0, vis);

    int y = 2;
    for (std::size_t index = 0; index < lines.size(); ++index)
    {
        int            baseline = 0;
        const cv::Size size     = cv::getTextSize(lines[index], cv::FONT_HERSHEY_SIMPLEX, font_scales[index], thickness, &baseline);
        y += size.height + 4;
        cv::putText(vis, lines[index], {6, y}, cv::FONT_HERSHEY_SIMPLEX, font_scales[index], (index == 1) ? green : yellow, thickness);
    }
    return vis;
}

void test_image(const std::string &path)
{
    using namespace rm::armor;

    const cv::Mat image = cv::imread(path);
    check(!image.empty(), "read image " + path);
    if (image.empty())
    {
        return;
    }
    const tools::config::Config config(test_config());
    Detector                    detector(load_detector_config(config));

    const auto   begin      = std::chrono::steady_clock::now();
    const auto   result     = detector.detect(image);
    const double latency_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();

    check(!result.armors.empty(), "detect finds armor(s) on " + path);
    check(!result.lightbars.empty(), "detect finds lightbar(s) on " + path);
    check(detector.detect(cv::Mat{}).armors.empty(), "empty frame tolerated");

    // ── 角点 / 灯条一致性 
    check(result.lightbars.size() == result.armors.size() * 2, "灯条数 = 2 x 装甲板数");
    for (std::size_t index = 0; index < result.armors.size(); ++index)
    {
        const auto       &armor = result.armors[index];
        const auto       &left  = result.lightbars[2 * index];
        const auto       &right = result.lightbars[(2 * index) + 1];
        const std::string tag   = " (i=" + std::to_string(index) + ")";
        check(close(left.upper, armor.corners[0]) && close(left.lower, armor.corners[3]), "左灯条端点 == corners[0]/[3]" + tag);
        check(close(right.upper, armor.corners[1]) && close(right.lower, armor.corners[2]), "右灯条端点 == corners[1]/[2]" + tag);
    }

    // 精修生效检查
    double length_ratio_sum = 0.0;
    int    length_ratio_n   = 0;
    {
        DetectorConfig raw_config         = load_detector_config(config);
        raw_config.light.refine_min_width = 1.0E9; // 灯条宽度不可能超过它 -> 不精修
        Detector   raw_detector(raw_config);
        const auto raw_result = raw_detector.detect(image);

        check(raw_result.lightbars.size() == result.lightbars.size(), "开关精修不改变灯条数量");
        double shift_sum = 0.0;
        int    moved     = 0;
        for (std::size_t index = 0; index < result.lightbars.size(); ++index)
        {
            const auto  &refined    = result.lightbars[index];
            const auto  &raw        = raw_result.lightbars[index];
            const double shift      = std::max(cv::norm(refined.upper - raw.upper), cv::norm(refined.lower - raw.lower));
            const double raw_length = std::max(cv::norm(raw.upper - raw.lower), 1.0);
            // 不变量:精修只把端点往回收,不能改变灯条朝向(交换 top/bottom 会立刻被这条抓住)
            const cv::Point2f raw_direction     = raw.upper - raw.lower;
            const cv::Point2f refined_direction = refined.upper - refined.lower;
            check(raw_direction.dot(refined_direction) > 0.0F, "精修不改变灯条朝向");
            shift_sum += shift;
            length_ratio_sum += cv::norm(refined.upper - refined.lower) / raw_length;
            ++length_ratio_n;
            if (shift > 0.5)
            {
                ++moved;
            }
        }
        std::cout << "  PCA 精修:改动 " << moved << "/" << result.lightbars.size() << " 根灯条, 端点位移平均 "
                  << (shift_sum / static_cast<double>(std::max<std::size_t>(result.lightbars.size(), 1))) << " px\n";
        check(moved > 0, "PCA 精修确实改动了端点");
    }

    const double mean_length_ratio = length_ratio_n > 0 ? length_ratio_sum / static_cast<double>(length_ratio_n) : 0.0;
    std::cout << "  灯条长/未精修长度 平均 " << mean_length_ratio << "(PCA 会把端点收到硬边缘上)\n";
    check(mean_length_ratio < 0.98, "精修后长度明显短于未精修(说明精修在生效)");

    // 打印 + 存图
    std::cout << "[image] " << path << " -> " << result.armors.size() << " armor(s), " << result.lightbars.size() << " lightbar(s) (" << latency_ms
              << " ms)\n";
    for (const auto &armor : result.armors)
    {
        check(armor.confidence > 0.5F, "装甲板置信度 > 0.5");
        std::cout << "  armor   conf=" << armor.confidence << " kind=" << kind_name(armor.kind) << " color=" << color_name(armor.color)
                  << " tl=" << armor.corners[0] << " tr=" << armor.corners[1] << " br=" << armor.corners[2] << " bl=" << armor.corners[3] << "\n";
    }
    for (const auto &lightbar : result.lightbars)
    {
        std::cout << "  lightbar kind=" << kind_name(lightbar.kind) << " color=" << color_name(lightbar.color) << " upper=" << lightbar.upper
                  << " lower=" << lightbar.lower << "\n";
    }

    const std::filesystem::path input(path);
    const std::string           output = (input.parent_path() / (input.stem().string() + "_detected.png")).string();
    check(cv::imwrite(output, annotate(image, result, latency_ms)), "write " + output);
    std::cout << "  output=" << output << "\n";
}

} // namespace

int main(int argc, char **argv) noexcept
{
    std::string input;
    for (int i = 1; i < argc; ++i)
    {
        const std::string argument = argv[i];
        if (input.empty())
        {
            input = argument;
        }
    }

    try
    {
        test_image(input.empty() ? std::string(RM_AUTO_ARMOR_MODELS_DIR) + "/test.png" : input);
    }
    catch (const std::exception &error)
    {
        std::cerr << "FAIL: exception " << error.what() << "\n";
        ++failures;
    }

    std::cout << (failures == 0 ? "auto_armor test passed\n" : "auto_armor test failed\n");
    return failures == 0 ? 0 : 1;
}
