#include "modules/auto_armor/auto_armor.hpp"
#include "modules/auto_armor/debug/visualize.hpp"
#include "modules/auto_armor/detection/detector.hpp"
#include "modules/auto_armor/detection/green_light.hpp"
#include "modules/auto_armor/models/shenzhen_model.hpp"
#include "tools/debug/debug.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

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

bool close(const cv::Point2f &a, const cv::Point2f &b, float tol = 1E-3F) { return std::abs(a.x - b.x) <= tol && std::abs(a.y - b.y) <= tol; }

// 构造一条输出:默认一条红色 3 号步兵,置信度 sigmoid(1.0)=0.731。
rm::armor::ShenZhenResult make_raw()
{
    rm::armor::ShenZhenResult raw{};
    raw.corners    = {100.0F, 110.0F, 90.0F, 200.0F, 190.0F, 210.0F, 200.0F, 120.0F};
    raw.confidence = 1.0F;
    raw.color      = {0.0F, 10.0F, 0.0F, 0.0F}; // blue, red, dark, mix
    raw.genre      = {0.0F, 0.0F, 0.0F, 5.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    return raw;
}

void test_decode()
{
    using namespace rm::armor;

    // 角点顺序 tl,tr,br,bl + letterbox 反算 p*0.5+(10,20)。
    const auto armor = decode_shenzhen(make_raw(), 0.5F, {10.0F, 20.0F}, 0.5F);
    check(armor.has_value(), "decode returns armor");
    if (armor)
    {
        check(close(armor->corners[0], {60.0F, 75.0F}), "tl mapped");
        check(close(armor->corners[1], {110.0F, 80.0F}), "tr mapped");
        check(close(armor->corners[2], {105.0F, 125.0F}), "br mapped");
        check(close(armor->corners[3], {55.0F, 120.0F}), "bl mapped");
        check(close(armor->center, {82.5F, 100.0F}), "center is corner mean");
        check(armor->kind == Kind::infantry1, "genre infantry_3 -> Infantry1");
        check(armor->color == Color::red, "color red -> Red");
        check(std::abs(armor->confidence - 0.73105858F) < 1E-4F, "confidence sigmoid");
    }

    // UNKNOWN:所有类别为 0 -> 丢弃。
    auto unknown  = make_raw();
    unknown.genre = {};
    check(!decode_shenzhen(unknown, 0.5F, {}, 1.0F).has_value(), "unknown genre dropped");

    // 置信度不足:sigmoid(-10) ~ 4.5e-5 <= 0.5。
    auto low       = make_raw();
    low.confidence = -10.0F;
    check(!decode_shenzhen(low, 0.5F, {}, 1.0F).has_value(), "low confidence dropped");

    // 颜色 dark -> Gray;类别 base_large -> Base。
    auto gray_base  = make_raw();
    gray_base.color = {0.0F, 0.0F, 7.0F, 0.0F};
    gray_base.genre = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 9.0F};
    const auto gb   = decode_shenzhen(gray_base, 0.5F, {}, 1.0F);
    check(gb.has_value() && gb->color == Color::gray, "dark -> Gray");
    check(gb.has_value() && gb->kind == Kind::base, "base_large -> Base");

    // 文件名识别。
    check(is_shenzhen_model("shenzhen-0526.onnx"), "0526 recognized");
    check(is_shenzhen_model("shenzhen-0708.onnx"), "0708 recognized");
    check(!is_shenzhen_model("other.onnx"), "unknown model rejected");

    // 通用解析器查找。
    check(find_model_spec("shenzhen-0526.onnx") != nullptr, "0526 spec found");
    check(find_model_spec("other.onnx") == nullptr, "unknown model has no spec");
}

// 绿灯滤除:合成一张带绿色圆点的图,验证绿灯上方建筑板被删、下方保留。
void test_green_light()
{
    using namespace rm::armor;

    cv::Mat image(400, 600, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::circle(image, {300, 100}, 12, cv::Scalar(20, 200, 20), cv::FILLED); // BGR 绿色

    Armor2d above; // 位于绿灯上方 -> 应删除
    above.corners = {{{280.0F, 20.0F}, {320.0F, 20.0F}, {320.0F, 60.0F}, {280.0F, 60.0F}}};
    above.kind    = Kind::outpost;

    Armor2d below; // 位于绿灯下方 -> 应保留
    below.corners = {{{280.0F, 160.0F}, {320.0F, 160.0F}, {320.0F, 200.0F}, {280.0F, 200.0F}}};
    below.kind    = Kind::base;

    std::vector<Armor2d> armors{above, below};
    const auto           result = find_green_light(image, armors, GreenLightConfig{});
    check(result.green_light.has_value(), "green light found");
    if (result.green_light)
    {
        check(result.green_light->y > 80 && result.green_light->y < 100, "green light near circle");
        filter_buildings_above_green_light(armors, *result.green_light);
    }
    check(armors.size() == 1, "only building below green light kept");
    check(!armors.empty() && armors[0].kind == Kind::base, "kept armor is below green light");

    check(GreenLightConfig{}.enable, "green light filter enabled by default");
}

// 颜色门控:解析配置色 + 只保留指定颜色。
void test_color_gate()
{
    using namespace rm::armor;

    check(parse_color("red") == Color::red, "parse red");
    check(parse_color("blue") == Color::blue, "parse blue");
    check(parse_color("gray") == Color::gray, "parse gray");
    check(parse_color("purple") == Color::purple, "parse purple");
    check(!parse_color("").has_value(), "parse empty -> no filter");
    check(!parse_color("bogus").has_value(), "parse unknown -> no filter");

    Armor2d red;
    red.color = Color::red;
    Armor2d blue;
    blue.color = Color::blue;
    Armor2d gray;
    gray.color = Color::gray;

    std::vector<Armor2d> armors{red, blue, gray};
    filter_by_color(armors, Color::red);
    check(armors.size() == 1 && armors[0].color == Color::red, "keep only enemy color");
}

#ifdef RM_DEBUG
// 识别颜色 -> BGR 显示色(红/蓝/灰/紫)。
cv::Scalar color_bgr(rm::armor::Color color)
{
    switch (color)
    {
    case rm::armor::Color::red:
        return {0, 0, 255};
    case rm::armor::Color::blue:
        return {255, 0, 0};
    case rm::armor::Color::gray:
        return {128, 128, 128};
    case rm::armor::Color::purple:
        return {255, 0, 255};
    }
    return {255, 255, 255};
}

// 把识别效果画到图上,Viewer 不在线时静默跳过。
void report_debug(const cv::Mat &image, const rm::armor::Detector::Result &result, double latency_ms)
{
    try
    {
        tools::debug::Sink sink("rm_vision.auto_armor_test");
        if (!sink.active())
        {
            return;
        }

        cv::Mat vis = image.clone();
        for (const auto &armor : result.armors)
        {
            std::vector<cv::Point> polygon;
            polygon.reserve(armor.corners.size());
            for (const auto &corner : armor.corners)
            {
                polygon.emplace_back(cvRound(corner.x), cvRound(corner.y));
            }
            const cv::Scalar color = color_bgr(armor.color);
            cv::polylines(vis, polygon, true, color, 2);
            const std::string label = "k" + std::to_string(static_cast<int>(armor.kind)) + " c" + std::to_string(static_cast<int>(armor.color)) +
                                      " " + cv::format("%.2f", armor.confidence);
            cv::putText(vis, label, polygon.front(), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        }

        std::vector<std::uint8_t> jpeg;
        cv::imencode(".jpg", vis, jpeg, {cv::IMWRITE_JPEG_QUALITY, 80});
        sink.image("detection/image", std::move(jpeg));
        sink.data("detection/latency_ms", latency_ms);
        std::cout << "[rerun] pushed detection image + latency (" << latency_ms << " ms)\n";
    }
    catch (const std::exception &error)
    {
        std::cout << "[rerun] skip: " << error.what() << "\n";
    }
}
#endif

// 在图上叠加识别结果 + 耗时 HUD
cv::Mat annotate(const cv::Mat &image, const rm::armor::Detector::Result &result, double latency_ms)
{
    cv::Mat vis = image.clone();
    rm::armor::draw(vis, result);
    const std::string hud = cv::format("armor %d | %.1f ms", static_cast<int>(result.armors.size()), latency_ms);
    cv::putText(vis, hud, {8, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
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

    DetectorConfig config;
    config.model  = std::string(RM_AUTO_ARMOR_MODELS_DIR) + "/shenzhen-0526.onnx";
    config.device = "CPU";
    AutoAim       auto_aim(config);

    const auto   begin      = std::chrono::steady_clock::now();
    const auto   result     = auto_aim.process(image);
    const double latency_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
    check(!result.detection.armors.empty(), "detect finds armor(s) on " + path);

    const std::filesystem::path input(path);
    const std::string           output = (input.parent_path() / (input.stem().string() + "_detected.png")).string();
    cv::imwrite(output, annotate(image, result.detection, latency_ms));

    std::cout << "[image] " << path << " -> " << result.detection.armors.size() << " armor(s) (" << latency_ms << " ms)\n";
    for (const auto &armor : result.detection.armors)
    {
        std::cout << "  conf=" << armor.confidence << " kind=" << static_cast<int>(armor.kind) << " color=" << static_cast<int>(armor.color)
                  << " corners=" << armor.corners[0] << armor.corners[1] << armor.corners[2] << armor.corners[3] << "\n";
    }

    std::cout << "  output=" << output << "\n";

    check(auto_aim.process(cv::Mat{}).detection.armors.empty(), "empty frame tolerated");

#ifdef RM_DEBUG
    report_debug(image, result.detection, latency_ms);
#endif
}

// load_detector_config 应把相对的 model 路径按仓库根解析成存在的绝对路径。
void test_config_path()
{
    const std::string yaml_path = (std::filesystem::temp_directory_path() / "rm_auto_armor_config_test.yaml").string();
    {
        std::ofstream out(yaml_path);
        out << "detector:\n  model: modules/auto_armor/models/shenzhen-0526.onnx\n";
    }

    const tools::config::Config     config(yaml_path);
    const rm::armor::DetectorConfig detector = rm::armor::load_detector_config(config);
    check(std::filesystem::path(detector.model).is_absolute(), "relative model path resolves to absolute");
    check(std::filesystem::exists(detector.model), "resolved model path exists: " + detector.model);
}

} // namespace

int main(int argc, char **argv)
{
    bool        self_test = false;
    std::string input;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--self-test")
        {
            self_test = true;
        }
        else if (input.empty())
        {
            input = arg;
        }
    }

    test_decode();
    test_green_light();
    test_color_gate();
    test_config_path();

    if (!self_test)
    {
        test_image(input.empty() ? std::string(RM_AUTO_ARMOR_MODELS_DIR) + "/test.png" : input);
    }

    std::cout << (failures == 0 ? "auto_armor test passed\n" : "auto_armor test failed\n");
    return failures == 0 ? 0 : 1;
}
