#include "modules/auto_buff/detection/detector.hpp"
#include "tools/config/config.hpp"
#include "tools/debug/debug.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

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

const char *kind_name(rm::buff::Kind kind)
{
    switch (kind)
    {
    case rm::buff::Kind::inactive:
        return "inactive";
    case rm::buff::Kind::small_activated:
        return "small_activated";
    case rm::buff::Kind::big_activated:
        return "big_activated";
    }
    return "unknown";
}

// 关键点固定配色,索引 2(R 标)用红色以便区分。
cv::Scalar keypoint_color(int index)
{
    static const std::array<cv::Scalar, rm::buff::KEYPOINT_COUNT> colors{
        cv::Scalar(0, 255, 0),   // top   green
        cv::Scalar(0, 255, 255), // left  yellow
        cv::Scalar(0, 0, 255),   // R     red
        cv::Scalar(255, 255, 0), // right cyan
        cv::Scalar(255, 0, 255)  // bottom magenta
    };
    return colors[static_cast<std::size_t>(index)];
}

cv::Mat annotate(const cv::Mat &image, const rm::buff::Detector::Result &result, int frame_index = -1, double latency_ms = 0.0)
{
    cv::Mat vis = image.clone();
    for (const auto &rune : result.runes)
    {
        for (int k = 0; k < rm::buff::KEYPOINT_COUNT; ++k)
        {
            const cv::Point point(cvRound(rune.keypoints[static_cast<std::size_t>(k)].x), cvRound(rune.keypoints[static_cast<std::size_t>(k)].y));
            cv::circle(vis, point, (k == rm::buff::KPT_R) ? 5 : 3, keypoint_color(k), cv::FILLED);
        }
        const cv::Point   anchor(cvRound(rune.keypoints[rm::buff::KPT_TOP].x), cvRound(rune.keypoints[rm::buff::KPT_TOP].y) - 8);
        const std::string label = std::string(kind_name(rune.kind)) + " " + cv::format("%.2f", rune.confidence);
        cv::putText(vis, label, anchor, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }
    if (frame_index >= 0)
    {
        const std::string hud = cv::format("frame %d | rune %d | %.1f ms", frame_index, static_cast<int>(result.runes.size()), latency_ms);
        cv::putText(vis, hud, {8, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    }
    return vis;
}

bool has_video_extension(const std::filesystem::path &path)
{
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    static const std::array<const char *, 7> exts{".mp4", ".avi", ".mkv", ".mov", ".flv", ".webm", ".m4v"};
    return std::any_of(exts.begin(), exts.end(), [&](const char *e) { return ext == e; });
}

bool has_image_extension(const std::filesystem::path &path)
{
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    static const std::array<const char *, 6> exts{".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"};
    return std::any_of(exts.begin(), exts.end(), [&](const char *e) { return ext == e; });
}

// 找出 test/ 目录下所有以 "test" 开头的视频或图片(跳过已生成的 *_detected.*)。
std::vector<std::string> find_test_inputs(const std::string &dir)
{
    std::vector<std::string> inputs;
    if (!std::filesystem::is_directory(dir))
    {
        return inputs;
    }
    for (const auto &entry : std::filesystem::directory_iterator(dir))
    {
        if (!entry.is_regular_file())
        {
            continue;
        }
        const auto       &path = entry.path();
        const std::string name = path.filename().string();
        if (name.find("_detected") != std::string::npos)
        {
            continue;
        }
        if (path.stem().string().rfind("test", 0) != 0)
        {
            continue;
        }
        if (!has_video_extension(path) && !has_image_extension(path))
        {
            continue;
        }
        inputs.push_back(path.string());
    }
    std::sort(inputs.begin(), inputs.end());
    return inputs;
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
        const cv::Mat             vis = annotate(image, result, -1, latency_ms);
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

void test_image(const std::string &path, const rm::buff::DetectorConfig &config)
{
    const cv::Mat image = cv::imread(path);
    check(!image.empty(), "read image " + path);
    if (image.empty())
    {
        return;
    }

    rm::buff::Detector detector(config);
    const auto         begin      = std::chrono::steady_clock::now();
    const auto         result     = detector.detect(image);
    const double       latency_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();

    const std::filesystem::path input(path);
    const std::string           output = (input.parent_path() / (input.stem().string() + "_detected.png")).string();
    cv::imwrite(output, annotate(image, result, -1, latency_ms));

    std::cout << "[image] " << path << " -> " << result.runes.size() << " rune(s) (" << latency_ms << " ms)\n";
    for (const auto &rune : result.runes)
    {
        std::cout << "  " << kind_name(rune.kind) << " conf=" << rune.confidence << " kpts=";
        for (const auto &kpt : rune.keypoints)
        {
            std::cout << "(" << kpt.x << "," << kpt.y << ")";
        }
        std::cout << "\n";
    }
    std::cout << "  output=" << output << "\n";
#ifdef RM_DEBUG
    report_debug(image, result, latency_ms);
#endif
}

void test_video(const std::string &path, const rm::buff::DetectorConfig &config)
{
    cv::VideoCapture capture(path);
    if (!capture.isOpened())
    {
        check(false, "open video " + path);
        return;
    }

    cv::Mat frame;
    if (!capture.read(frame) || frame.empty())
    {
        check(false, "read first frame " + path);
        return;
    }

    const double                fps_raw = capture.get(cv::CAP_PROP_FPS);
    const double                fps     = (fps_raw > 0.0 && fps_raw <= 240.0) ? fps_raw : 25.0;
    const std::filesystem::path input(path);

    // 优先 mp4v/.mp4,失败回退 XVID/.avi、MJPG/.avi。
    std::string     output = (input.parent_path() / (input.stem().string() + "_detected.mp4")).string();
    cv::VideoWriter writer;
    bool            opened = writer.open(output, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, frame.size());
    if (!opened)
    {
        output = (input.parent_path() / (input.stem().string() + "_detected.avi")).string();
        opened = writer.open(output, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps, frame.size());
    }
    if (!opened)
    {
        opened = writer.open(output, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, frame.size());
    }
    if (!opened)
    {
        check(false, "open writer for " + output);
        return;
    }

    rm::buff::Detector detector(config);
    int                index            = 0;
    int                frames_with_rune = 0;
    int                total_runes      = 0;
    int                total_active     = 0;
    double             total_ms         = 0.0;

    while (!frame.empty())
    {
        const auto   begin      = std::chrono::steady_clock::now();
        const auto   result     = detector.detect(frame);
        const double latency_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
        total_ms += latency_ms;

        if (!result.runes.empty())
        {
            ++frames_with_rune;
            total_runes += static_cast<int>(result.runes.size());
            for (const auto &rune : result.runes)
            {
                if (rune.kind != rm::buff::Kind::inactive)
                {
                    ++total_active;
                }
            }
        }

        writer.write(annotate(frame, result, index, latency_ms));
        if (index % 50 == 0)
        {
            std::cout << "  [video] frame " << index << " runes=" << result.runes.size() << " " << latency_ms << " ms\n";
        }
        ++index;
        if (!capture.read(frame))
        {
            break;
        }
    }

    const double avg_ms = (index > 0) ? total_ms / index : 0.0;
    std::cout << "[video] " << path << "\n"
              << "  frames=" << index << " with_rune=" << frames_with_rune << " runes=" << total_runes << " active=" << total_active << "\n"
              << "  avg_latency=" << avg_ms << " ms (" << (avg_ms > 0.0 ? 1000.0 / avg_ms : 0.0) << " fps)\n"
              << "  output=" << output << "\n";
}

// load_detector_config 应把相对的 model 路径按仓库根解析成存在的绝对路径。
void test_config()
{
    const std::string yaml_path = (std::filesystem::temp_directory_path() / "rm_auto_buff_config_test.yaml").string();
    {
        std::ofstream out(yaml_path);
        out << "buff:\n  model: modules/auto_buff/models/shenzhenbuff-0624.onnx\n";
    }

    const tools::config::Config    config(yaml_path);
    const rm::buff::DetectorConfig detector = rm::buff::load_detector_config(config);
    check(std::filesystem::path(detector.model).is_absolute(), "relative model path resolves to absolute");
    check(std::filesystem::exists(detector.model), "resolved model path exists: " + detector.model);
}

// 合成图跑通「加载模型 -> 预处理 -> 推理 -> 后处理」全链路,并验证空图安全。
void test_model_smoke()
{
    try
    {
        rm::buff::Detector detector(make_config());
        const cv::Mat      image(1080, 1440, CV_8UC3, cv::Scalar(40, 40, 40));
        const auto         result = detector.detect(image);
        check(true, "smoke ok, runes=" + std::to_string(result.runes.size()));
        check(detector.detect(cv::Mat{}).runes.empty(), "empty image returns empty");
    }
    catch (const std::exception &error)
    {
        check(false, std::string("smoke threw: ") + error.what());
    }
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
    test_model_smoke();

    if (!self_test)
    {
        const rm::buff::DetectorConfig config = make_config();
        if (input.empty())
        {
            for (const auto &item : find_test_inputs(RM_AUTO_BUFF_TEST_DIR))
            {
                if (has_video_extension(item))
                {
                    test_video(item, config);
                }
                else
                {
                    test_image(item, config);
                }
            }
        }
        else if (has_video_extension(input))
        {
            test_video(input, config);
        }
        else
        {
            test_image(input, config);
        }
    }

    std::cout << (failures == 0 ? "auto_buff test passed\n" : "auto_buff test failed\n");
    return failures == 0 ? 0 : 1;
}
