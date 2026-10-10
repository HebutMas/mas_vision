#include "modules/auto_buff/auto_buff.hpp"
#include "modules/auto_buff/debug/visualize.hpp"
#include "modules/auto_buff/detection/refiner.hpp"
#include "tools/config/config.hpp"
#include "tools/debug/debug.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

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

// 默认用仓库内的五点模型;帧率/置信度等走结构体默认值。
rm::buff::DetectorConfig make_config()
{
    rm::buff::DetectorConfig config;
    config.model  = std::string(RM_AUTO_BUFF_MODELS_DIR) + "/shenzhenbuff-0624.onnx";
    config.device = "CPU";
    return config;
}

#ifdef RM_DEBUG
// 把识别效果画到图上,Viewer 不在线时静默跳过。
void report_debug(const cv::Mat &image, const rm::buff::Detector::Result &result, double latency_ms)
{
    try
    {
        tools::debug::Sink sink("rm_vision.auto_buff_test");
        if (!sink.active())
        {
            return;
        }
        cv::Mat             vis = image.clone();
        rm::buff::draw(vis, result);
        std::vector<std::uint8_t> jpeg;
        cv::imencode(".jpg", vis, jpeg, {cv::IMWRITE_JPEG_QUALITY, 80});
        sink.image("buff/image", std::move(jpeg));
        sink.data("buff/latency_ms", latency_ms);
    }
    catch (const std::exception &error)
    {
        std::cout << "[rerun] skip: " << error.what() << "\n";
    }
}
#endif


void test_image(const std::string &path)
{
    using namespace rm::buff;

    const cv::Mat image = cv::imread(path);
    check(!image.empty(), "read image " + path);
    if (image.empty())
    {
        return;
    }

    AutoBuff       auto_buff(make_config());
    const auto     begin      = std::chrono::steady_clock::now();
    const auto     result     = auto_buff.process(image);
    const double   latency_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();

    check(!result.detection.runes.empty(), "detect finds rune(s) on " + path);

    const std::filesystem::path input(path);
    const std::string           output = (input.parent_path() / (input.stem().string() + "_detected.png")).string();
    cv::Mat                    vis    = image.clone();
    rm::buff::draw(vis, result.detection);
    cv::imwrite(output, vis);

    std::cout << "[image] " << path << " -> " << result.detection.runes.size() << " rune(s) (" << latency_ms << " ms)\n";
    for (const auto &rune : result.detection.runes)
    {
        std::cout << "  " << kind_name(rune.kind) << " " << color_name(rune.color) << " conf=" << rune.confidence << " kpts=";
        for (const auto &kpt : rune.keypoints)
        {
            std::cout << "(" << kpt.x << "," << kpt.y << ")";
        }
        std::cout << "\n";
    }
    std::cout << "  target=" << (result.target ? kind_name(result.target->kind) : "none") << "\n";
    std::cout << "  output=" << output << "\n";

    check(auto_buff.process(cv::Mat{}).detection.runes.empty(), "empty frame tolerated");

#ifdef RM_DEBUG
    report_debug(image, result.detection, latency_ms);
#endif
}

// load_detector_config 应把相对的 model 路径按仓库根解析成存在的绝对路径。
void test_config()
{
    const std::string yaml_path = (std::filesystem::temp_directory_path() / "rm_auto_buff_config_test.yaml").string();
    {
        std::ofstream out(yaml_path);
        out << "auto_buff:\n  detector:\n    model: modules/auto_buff/models/shenzhenbuff-0624.onnx\n    refine:\n      border_margin: 3.5\n";
    }

    const tools::config::Config    config(yaml_path);
    const rm::buff::DetectorConfig detector = rm::buff::load_detector_config(config);
    check(std::filesystem::path(detector.model).is_absolute(), "relative model path resolves to absolute");
    check(std::filesystem::exists(detector.model), "resolved model path exists: " + detector.model);
    check(detector.refiner.border_margin > 3.0, "auto_buff.detector.refine parsed");
}

// 合成红方符叶:中心大圆=装甲板,左侧长条=未激活灯臂,最左小圆=中心 R,
// 验证三类轮廓能被区分且描述符判定为可用。
void test_refiner()
{
    cv::Mat image(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::circle(image, {320, 320}, 120, cv::Scalar(0, 0, 255), cv::FILLED);
    cv::rectangle(image, {110, 310}, {190, 330}, cv::Scalar(0, 0, 255), cv::FILLED);
    cv::circle(image, {80, 320}, 12, cv::Scalar(0, 0, 255), cv::FILLED);

    rm::buff::Rune2d rune;
    rune.kind                            = rm::buff::Kind::inactive;
    rune.color                           = rm::buff::Color::red;
    rune.keypoints[rm::buff::kpt_top]    = {320.0F, 200.0F};
    rune.keypoints[rm::buff::kpt_left]   = {200.0F, 320.0F};
    rune.keypoints[rm::buff::kpt_r]      = {80.0F, 320.0F};
    rune.keypoints[rm::buff::kpt_right]  = {440.0F, 320.0F};
    rune.keypoints[rm::buff::kpt_bottom] = {320.0F, 440.0F};

    rm::buff::RefinerConfig        config;
    config.roi_margin_ratio        = 0.6;
    const rm::buff::RuneRefinement result = rm::buff::refine(image, rune, config);
    check(result.is_armor_module_usable, "refiner: armor module usable");
    check(result.is_light_arm_usable, "refiner: light arm usable");
    check(result.is_center_r_usable, "refiner: center R usable");
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

    test_config();
    test_refiner();

    if (!self_test)
    {
        test_image(input.empty() ? std::string(RM_AUTO_BUFF_MODELS_DIR) + "/test.png" : input);
    }

    std::cout << (failures == 0 ? "auto_buff test passed\n" : "auto_buff test failed\n");
    return failures == 0 ? 0 : 1;
}