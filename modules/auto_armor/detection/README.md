# detection 装甲板检测

基于 shenzhen 的 4 点 keypoint 模型，用OpenVINO 在 CPU 上推理,输出装甲板的四角、颜色、类别和置信度。

## 处理流水线

`Detector::detect(bgr)` 的内部流程:

```
BGR 图
  → (可选) ROI 裁剪,记下 offset
  → letterbox:等比缩放到左上角,右下补零到 input_w × input_h
  → OpenVINO 推理(u8 NHWC BGR → f32 RGB /255,由 PrePostProcessor 完成)
  → 逐 anchor 解码:confidence 过 sigmoid 与 min_confidence,color/genre 取 argmax,
     角点做反算 p * (1/scale) + offset
  → NMSBoxes(score_threshold, nms_threshold)
  → (可选) 颜色门控 enemy_color
  → 灯条角点优化 -> refine_lightbar() 精修 -> 写回 armor.corners + 成对放进 lightbars
  → Result{armors, lightbars}
```

- **letterbox** 只缩放、不居中,多余区域补 0
- **角点顺序** 固定为 `tl, tr, br, bl`(像素坐标),模型原始顺序 `lt, lb, rb, rt` 在解码时转换。

## 文件职责

| 文件 | 说明 |
|---|---|
| `detector.hpp` / `detector.cpp` | 通用 `Detector`:模型加载、预处理、推理、NMS、ROI、灯条提取与后处理编排 |
| `lightbar.hpp` / `lightbar.cpp` | 灯条角点优化:`refine_lightbar()` |
| `../models/model.hpp` / `model.cpp` | 模型输出解析器接口 `ModelSpec` 与注册表 `find_model_spec` |
| `../models/shenzhen_model.hpp` / `shenzhen_model.cpp` | shenzhen 模型的输出布局与解码实现 |
| `../models/shenzhen-0526.onnx` / `shenzhen-0708.onnx` | 模型权重 |
| `../models/test.png` | 测试用例图 |
| `../../test.cpp` | 主测试:用 `Detector` 跑 `models/test.png`,画框/角点/灯条并存图 |

## 接口

```cpp
struct DetectorConfig
{
    std::string model;                                  // onnx 路径,必填
    std::string device{"CPU"};                          // OpenVINO 设备
    int         input_w{640}, input_h{640};             // 输入尺寸,须与模型静态形状一致
    float       min_confidence{0.5F};                   // 解码阶段单框置信度下限
    float       score_threshold{0.7F};                  // NMS 得分阈值
    float       nms_threshold{0.3F};                    // NMS IoU 阈值
    bool        use_roi{false};                         // 是否只在 roi 内检测
    cv::Rect    roi;                                    // use_roi 为 true 时生效,自动与图像求交
    std::optional<Color> enemy_color;                   // 颜色门控,nullopt = 不过滤
    LightRefineParams    light;                         // 灯条优化参数
};

[[nodiscard]] DetectorConfig load_detector_config(const tools::config::Config &config);

class Detector
{
  public:
    explicit Detector(const DetectorConfig &config);
    // lightbars:第 i 块装甲板的两条 = [2i](左,tl->bl)、[2i+1](右,tr->br),一一对应。
    struct Result { std::vector<Armor2d> armors; std::vector<Lightbar2d> lightbars; };
    Result detect(const cv::Mat &bgr);
};
```

- `Detector` 是 move-only(pimpl),不可拷贝。
- 构造时按文件名查 `find_model_spec`;未注册的模型抛 `unsupported armor model: <name>`。
- 模型输入为静态形状且与 `input_w/input_h` 不符时,抛 `detector input size mismatch`。
- `Armor2d::corners` 为 `tl, tr, br, bl`;`center` 为四角均值。

## 配置

`apps/infantry/config.yaml` 的 `detector` 段:

```yaml
detector:
  model: modules/auto_armor/models/shenzhen-0526.onnx # 相对仓库根目录
  device: CPU
  input: [640, 640]
  min_confidence: 0.5
  score_threshold: 0.7
  nms_threshold: 0.3
  roi: {enable: false, x: 420, y: 50, width: 600, height: 600}
  enemy_color: "" # 颜色判断,空代表都识别
  light: # 灯条优化参数
    refine_min_width_px: 3.0
    refine_start_ratio: 0.4
    refine_end_ratio: 0.6
```

| 键 | 默认 | 说明 |
|---|---|---|
| `detector.model` | 无(必填) | onnx 路径 |
| `detector.device` | `CPU` | OpenVINO 设备字符串 |
| `detector.input` | `[640, 640]` | `[width, height]`,长度不为 2 时回退 640×640 |
| `detector.min_confidence` | `0.5` | 单框置信度下限 |
| `detector.score_threshold` | `0.7` | NMS 得分阈值 |
| `detector.nms_threshold` | `0.3` | NMS IoU 阈值 |
| `detector.roi.enable` | `false` | 是否用 ROI |
| `detector.roi.x/y/width/height` | `0` | ROI 区域 |
| `detector.enemy_color` | `""` | 颜色门控,`red`/`blue`/`gray`/`purple`;空 = 不过滤 |
| `detector.light.refine_min_width_px` | `3.0` | 最小灯条宽度 |
| `detector.light.refine_start_ratio` / `refine_end_ratio` | `0.4` / `0.6` | 沿对称轴的搜索窗口 |

## shenzhen 模型输出布局

每块装甲板对应 **22 个 float**:

| 字段 | 数量 | 含义 |
|---|---|---|
| `corners` | 8 | `lt_x, lt_y, lb_x, lb_y, rb_x, rb_y, rt_x, rt_y` |
| `confidence` | 1 | 原始 logit,解码时过 sigmoid |
| `color` | 4 | `blue, red, dark, mix` → `blue, red, gray, purple` |
| `genre` | 9 | `sentry, hero, engineer, infantry_3, infantry_4, infantry_5, outpost, base_small, base_large` |

`genre` 额外含一个全 0 的 UNKNOWN 类别(下标 0),解码时直接丢弃。


## 扩展新模型

1. 新增 `models/<name>_model.hpp` / `.cpp`,给出 `floats_per_detection` 与 `decode` 函数;
2. 在 `models/model.cpp` 的 `find_model_spec` 里登记一行;
3. 在yaml文件里onnx路径切换为新模型
