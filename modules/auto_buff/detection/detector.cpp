#include "modules/auto_buff/detection/detector.hpp"

#include "modules/auto_buff/models/model.hpp"

#include <algorithm>
#include <filesystem>
#include <numeric>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <openvino/core/preprocess/pre_post_process.hpp>
#include <openvino/runtime/compiled_model.hpp>
#include <openvino/runtime/core.hpp>

namespace rm::buff
{

namespace
{
[[nodiscard]] float squared_distance(const cv::Point2f &a, const cv::Point2f &b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return (dx * dx) + (dy * dy);
}
} // namespace

DetectorConfig load_detector_config(const tools::config::Config &config)
{
    DetectorConfig cfg;
    cfg.model = config.require<std::string>("buff.model");
    // model路径
    if (!cfg.model.empty() && std::filesystem::path(cfg.model).is_relative())
    {
        cfg.model = (std::filesystem::path(RM_AUTO_BUFF_ROOT) / cfg.model).string();
    }
    cfg.device                        = config.value<std::string>("buff.device", "CPU");
    cfg.confidence_threshold          = static_cast<float>(config.value<double>("buff.confidence_threshold", 0.8));
    cfg.keypoint_confidence_threshold = static_cast<float>(config.value<double>("buff.keypoint_confidence_threshold", 0.8));
    cfg.nms_distance_threshold        = static_cast<float>(config.value<double>("buff.nms_distance_threshold", 30.0));
    cfg.min_valid_keypoints           = config.value<int>("buff.min_valid_keypoints", 3);
    return cfg;
}

struct Detector::Impl
{
    explicit Impl(DetectorConfig config) : config_(std::move(config))
    {
        const std::string filename = std::filesystem::path(config_.model).filename().string();
        // 模型相关细节由 models 层提供,Detector 只做通用 I/O 与后处理。
        spec_ = find_model_spec(filename);
        if (spec_ == nullptr)
        {
            throw std::runtime_error("unsupported rune model: " + filename);
        }

        auto       raw         = core_.read_model(config_.model);
        const auto input_shape = raw->input().get_shape();
        if (input_shape.size() != 4)
        {
            throw std::runtime_error("rune model input must be a 4D NCHW tensor");
        }
        input_h_ = input_shape.at(2);
        input_w_ = input_shape.at(3);

        // 与训练预处理一致:u8 NHWC BGR -> f32 NCHW RGB,scale 1/255。
        ov::preprocess::PrePostProcessor ppp(raw);
        {
            auto &input = ppp.input();
            input.tensor().set_element_type(ov::element::u8).set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::BGR);
            input.preprocess().convert_element_type(ov::element::f32).convert_color(ov::preprocess::ColorFormat::RGB).scale({255.0, 255.0, 255.0});
            input.model().set_layout("NCHW");
        }
        ppp.output().tensor().set_element_type(ov::element::f32);

        model_   = core_.compile_model(ppp.build(), config_.device, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
        request_ = model_.create_infer_request();

        // 输入张量只分配一次并绑定到 request,每帧原地写入,省去逐帧分配 + set_input_tensor 的隐式拷贝。
        input_tensor_ = ov::Tensor(ov::element::u8, ov::Shape{1, input_h_, input_w_, 3});
        request_.set_input_tensor(input_tensor_);
    }

    [[nodiscard]] Result detect(const cv::Mat &bgr, Color color)
    {
        Result result;
        if (bgr.empty())
        {
            return result;
        }

        // 缩放到模型分辨率:按模型宽高比居中裁剪后再等比缩放
        // 4:3 相机(如 1440x1080)比例一致,等价于整帧缩放;其他比例(如 16:9)居中裁剪,
        // 避免出现训练时未见的填充灰边,同时保持几何不变形。
        const auto  target_w      = static_cast<int>(input_w_);
        const auto  target_h      = static_cast<int>(input_h_);
        const float target_aspect = static_cast<float>(target_w) / static_cast<float>(target_h);
        const float source_aspect = static_cast<float>(bgr.cols) / static_cast<float>(bgr.rows);
        int         crop_x        = 0;
        int         crop_y        = 0;
        int         crop_w        = bgr.cols;
        int         crop_h        = bgr.rows;
        if (source_aspect > target_aspect)
        {
            crop_w = std::min(bgr.cols, static_cast<int>(static_cast<float>(bgr.rows) * target_aspect));
            crop_x = (bgr.cols - crop_w) / 2;
        }
        else if (source_aspect < target_aspect)
        {
            crop_h = std::min(bgr.rows, static_cast<int>(static_cast<float>(bgr.cols) / target_aspect));
            crop_y = (bgr.rows - crop_h) / 2;
        }
        if (crop_w <= 0 || crop_h <= 0)
        {
            return result;
        }
        const float scale_x = static_cast<float>(target_w) / static_cast<float>(crop_w);
        const float scale_y = static_cast<float>(target_h) / static_cast<float>(crop_h);

        {
            cv::Mat       canvas(target_h, target_w, CV_8UC3, input_tensor_.data());
            const cv::Mat roi = bgr(cv::Rect(crop_x, crop_y, crop_w, crop_h));
            cv::resize(roi, canvas, {target_w, target_h});
        }

        request_.infer();

        const auto  output = request_.get_output_tensor();
        const auto &shape  = output.get_shape();
        if (shape.size() != 3)
        {
            return result;
        }

        // 输出可能是 [N,C,A] 或 [N,A,C],以通道数区分。
        const std::size_t dim1          = shape.at(1);
        const std::size_t dim2          = shape.at(2);
        const bool        channel_first = dim1 <= 256 && dim2 >= 100;
        const std::size_t channels      = channel_first ? dim1 : dim2;
        const std::size_t anchors       = channel_first ? dim2 : dim1;
        if (channels != spec_->channels)
        {
            return result;
        }

        const ModelOutput view{output.data<const float>(), channels, anchors, channel_first};

        DecodeParams params;
        params.crop_offset                   = {static_cast<float>(crop_x), static_cast<float>(crop_y)};
        params.inv_scale_x                   = 1.0F / scale_x;
        params.inv_scale_y                   = 1.0F / scale_y;
        params.confidence_threshold          = config_.confidence_threshold;
        params.keypoint_confidence_threshold = config_.keypoint_confidence_threshold;
        params.min_valid_keypoints           = config_.min_valid_keypoints;
        params.image_size                    = bgr.size();

        std::vector<Rune2d> candidates;
        spec_->decode(view, params, candidates);
        if (candidates.empty())
        {
            return result;
        }

        // 中心距 NMS:中心取有效关键点均值,质量取 类别分 × 平均关键点分。
        std::vector<cv::Point2f> centers;
        std::vector<float>       qualities;
        centers.reserve(candidates.size());
        qualities.reserve(candidates.size());
        for (auto &rune : candidates)
        {
            rune.color = color;
            cv::Point2f sum{0.0F, 0.0F};
            int         valid_kpts = 0;
            float       kconf_sum  = 0.0F;
            for (int k = 0; k < KEYPOINT_COUNT; ++k)
            {
                kconf_sum += rune.keypoint_confidence[k];
                if (rune.keypoint_confidence[k] >= config_.keypoint_confidence_threshold)
                {
                    ++valid_kpts;
                    sum += rune.keypoints[k];
                }
            }
            const float denom = (valid_kpts > 0) ? static_cast<float>(valid_kpts) : 1.0F;
            centers.push_back(sum / denom);
            qualities.push_back(rune.confidence * (kconf_sum / static_cast<float>(KEYPOINT_COUNT)));
        }

        std::vector<std::size_t> order(candidates.size());
        std::iota(order.begin(), order.end(), std::size_t{0});
        std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return qualities[a] > qualities[b]; });

        const float       nms_sq = config_.nms_distance_threshold * config_.nms_distance_threshold;
        std::vector<bool> suppressed(candidates.size(), false);
        result.runes.reserve(candidates.size());
        for (const std::size_t i : order)
        {
            if (suppressed[i])
            {
                continue;
            }
            for (std::size_t j = 0; j < candidates.size(); ++j)
            {
                if (!suppressed[j] && j != i && squared_distance(centers[i], centers[j]) < nms_sq)
                {
                    suppressed[j] = true;
                }
            }
            result.runes.push_back(candidates[i]);
        }
        return result;
    }

    DetectorConfig    config_;
    const ModelSpec  *spec_{};
    ov::Core          core_;
    ov::CompiledModel model_;
    ov::InferRequest  request_;
    ov::Tensor        input_tensor_;
    std::size_t       input_w_{0};
    std::size_t       input_h_{0};
};

Detector::Detector(const DetectorConfig &config) : impl_(std::make_unique<Impl>(config)) {}

Detector::~Detector()                               = default;
Detector::Detector(Detector &&) noexcept            = default;
Detector &Detector::operator=(Detector &&) noexcept = default;

Detector::Result Detector::detect(const cv::Mat &bgr, Color color) { return impl_->detect(bgr, color); }

} // namespace rm::buff
