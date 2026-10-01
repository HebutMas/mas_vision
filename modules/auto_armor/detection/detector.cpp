#include "modules/auto_armor/detection/detector.hpp"

#include "modules/auto_armor/models/model.hpp"

#include <algorithm>
#include <filesystem>
#include <optional>
#include <stdexcept>

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <openvino/core/preprocess/pre_post_process.hpp>
#include <openvino/runtime/compiled_model.hpp>
#include <openvino/runtime/core.hpp>

namespace rm::armor
{

DetectorConfig load_detector_config(const tools::config::Config &config)
{
    DetectorConfig cfg;
    cfg.model = config.require<std::string>("detector.model");
    // model 允许相对路径:相对仓库根解析(不依赖启动时的工作目录)。
    if (!cfg.model.empty() && std::filesystem::path(cfg.model).is_relative())
    {
        cfg.model = (std::filesystem::path(RM_AUTO_ARMOR_ROOT) / cfg.model).string();
    }
    cfg.device          = config.value<std::string>("detector.device", "CPU");
    cfg.min_confidence  = static_cast<float>(config.value<double>("detector.min_confidence", 0.5));
    cfg.score_threshold = static_cast<float>(config.value<double>("detector.score_threshold", 0.7));
    cfg.nms_threshold   = static_cast<float>(config.value<double>("detector.nms_threshold", 0.3));

    // input 为 [width, height];缺省或长度不符时保持 640x640。
    const auto input = config.value<std::vector<int>>("detector.input", {640, 640});
    if (input.size() == 2)
    {
        cfg.input_w = input[0];
        cfg.input_h = input[1];
    }

    cfg.use_roi    = config.value<bool>("detector.roi.enable", false);
    cfg.roi.x      = config.value<int>("detector.roi.x", 0);
    cfg.roi.y      = config.value<int>("detector.roi.y", 0);
    cfg.roi.width  = config.value<int>("detector.roi.width", 0);
    cfg.roi.height = config.value<int>("detector.roi.height", 0);

    cfg.green_light.enable           = config.value<bool>("detector.green_light.enable", true);
    cfg.green_light.green_threshold  = config.value<int>("detector.green_light.green_threshold", 120);
    cfg.green_light.min_area         = config.value<double>("detector.green_light.min_area", 20.0);
    cfg.green_light.min_circularity  = config.value<double>("detector.green_light.min_circularity", 0.6);
    cfg.green_light.max_aspect_ratio = config.value<double>("detector.green_light.max_aspect_ratio", 1.5);

    // 敌方颜色
    cfg.enemy_color = parse_color(config.value<std::string>("detector.enemy_color", ""));
    return cfg;
}

struct Detector::Impl
{
    explicit Impl(const DetectorConfig &config) : config(config)
    {
        const std::string filename = std::filesystem::path(config.model).filename().string();
        // 模型相关细节由 models 层提供,Detector 只做通用 I/O 与后处理。
        spec_ = find_model_spec(filename);
        if (spec_ == nullptr)
        {
            throw std::runtime_error("unsupported armor model: " + filename);
        }

        auto raw = core.read_model(config.model);
        // 模型输入为固定尺寸(NCHW),配置不符时报错
        const auto model_shape = raw->input().get_partial_shape();
        if (model_shape.is_static())
        {
            const auto shape = model_shape.to_shape();
            if (shape.size() == 4 &&
                (shape.at(2) != static_cast<std::size_t>(config.input_h) || shape.at(3) != static_cast<std::size_t>(config.input_w)))
            {
                throw std::runtime_error("detector input size mismatch: model expects " + std::to_string(shape.at(3)) + "x" +
                                         std::to_string(shape.at(2)) + ", config has " + std::to_string(config.input_w) + "x" +
                                         std::to_string(config.input_h));
            }
        }
        ov::preprocess::PrePostProcessor ppp(raw);
        {
            auto &input = ppp.input();
            input.tensor()
                .set_element_type(ov::element::u8)
                .set_shape(ov::PartialShape{1, config.input_h, config.input_w, 3})
                .set_layout("NHWC")
                .set_color_format(ov::preprocess::ColorFormat::BGR);
            input.preprocess().convert_element_type(ov::element::f32).convert_color(ov::preprocess::ColorFormat::RGB).scale({255.0, 255.0, 255.0});
            input.model().set_layout("NCHW");
        }
        ppp.output().tensor().set_element_type(ov::element::f32);

        model = core.compile_model(ppp.build(), config.device, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
    }

    Result detect(const cv::Mat &bgr)
    {
        Result result;
        if (bgr.empty())
        {
            return result;
        }

        cv::Mat     segmentation = bgr;
        cv::Point2f offset{0.0F, 0.0F};
        if (config.use_roi)
        {
            const cv::Rect roi = config.roi & cv::Rect(0, 0, bgr.cols, bgr.rows);
            if (roi.width <= 0 || roi.height <= 0)
            {
                return result;
            }
            segmentation = bgr(roi);
            offset       = {static_cast<float>(roi.x), static_cast<float>(roi.y)};
        }

        // letterbox:等比缩放到左上角,右下补零。
        const int   rows     = config.input_h;
        const int   cols     = config.input_w;
        const float scale    = std::min(static_cast<float>(cols) / segmentation.cols, static_cast<float>(rows) / segmentation.rows);
        const int   scaled_w = std::min(cols, static_cast<int>(segmentation.cols * scale));
        const int   scaled_h = std::min(rows, static_cast<int>(segmentation.rows * scale));
        if (scaled_w <= 0 || scaled_h <= 0)
        {
            return result;
        }

        auto input_tensor = ov::Tensor(ov::element::u8, ov::Shape{1, static_cast<std::size_t>(rows), static_cast<std::size_t>(cols), 3});
        {
            cv::Mat input_mat(rows, cols, CV_8UC3, input_tensor.data());
            input_mat.setTo(cv::Scalar::all(0));
            cv::resize(segmentation, input_mat(cv::Rect(0, 0, scaled_w, scaled_h)), {scaled_w, scaled_h});
        }

        auto request = model.create_infer_request();
        request.set_input_tensor(input_tensor);
        request.infer();

        const auto  output = request.get_output_tensor();
        const auto &shape  = output.get_shape();
        if (shape.size() != 3 || shape.at(2) != spec_->floats_per_detection)
        {
            return result;
        }

        const std::size_t anchors   = shape.at(1);
        const auto       *data      = output.data<const float>();
        const float       inv_scale = 1.0F / scale;

        std::vector<Armor2d>  candidates;
        std::vector<cv::Rect> boxes;
        std::vector<float>    scores;
        candidates.reserve(anchors);

        for (std::size_t i = 0; i < anchors; ++i)
        {
            const auto armor = spec_->decode(data + (i * spec_->floats_per_detection), config.min_confidence, offset, inv_scale);
            if (!armor)
            {
                continue;
            }
            candidates.push_back(*armor);
            scores.push_back(armor->confidence);
            boxes.push_back(cv::boundingRect(std::vector<cv::Point2f>(armor->corners.begin(), armor->corners.end())));
        }

        std::vector<int> kept;
        cv::dnn::NMSBoxes(boxes, scores, config.score_threshold, config.nms_threshold, kept);
        result.armors.reserve(kept.size());
        for (const int idx : kept)
        {
            result.armors.push_back(candidates[static_cast<std::size_t>(idx)]);
        }

        // 只保留敌方颜色
        if (config.enemy_color)
        {
            filter_by_color(result.armors, *config.enemy_color);
        }

        // 绿灯滤除:仅在识别到建筑类(前哨站 / 基地)时触发
        const bool has_building = std::any_of(result.armors.begin(), result.armors.end(),
                                              [](const Armor2d &armor) { return armor.kind == Kind::outpost || armor.kind == Kind::base; });
        if (config.green_light.enable && has_building)
        {
            std::vector<Armor2d> base;
            std::vector<Armor2d> outpost;
            for (const auto &armor : result.armors)
            {
                if (armor.kind == Kind::base)
                {
                    base.push_back(armor);
                }
                else if (armor.kind == Kind::outpost)
                {
                    outpost.push_back(armor);
                }
            }

            // 分开搜索:同时出现基地与前哨站时,合并 ROI 会过大。
            std::optional<cv::Rect> green_light;
            if (!base.empty())
            {
                green_light = find_green_light(bgr, base, config.green_light).green_light;
            }
            if (!outpost.empty())
            {
                const auto found = find_green_light(bgr, outpost, config.green_light).green_light;
                if (found)
                {
                    green_light = found;
                }
            }
            if (green_light)
            {
                filter_buildings_above_green_light(result.armors, *green_light);
            }
        }
        return result;
    }

    DetectorConfig    config;
    const ModelSpec  *spec_{};
    ov::Core          core;
    ov::CompiledModel model;
};

Detector::Detector(const DetectorConfig &config) : impl_(std::make_unique<Impl>(config)) {}

Detector::~Detector()                               = default;
Detector::Detector(Detector &&) noexcept            = default;
Detector &Detector::operator=(Detector &&) noexcept = default;

Detector::Result Detector::detect(const cv::Mat &bgr) { return impl_->detect(bgr); }

} // namespace rm::armor
