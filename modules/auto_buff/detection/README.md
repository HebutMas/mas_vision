# detection 能量机关检测

基于深大的 5 点 keypoint 模型。

## 处理流水线

`Detector::detect(bgr, color)` 的内部流程:

```
BGR 图
  → 按模型宽高比居中裁剪(模型 640×480 为 4:3),再 resize 到 input_w × input_h
  → OpenVINO 推理(u8 NHWC BGR → f32 RGB /255,由 PrePostProcessor 完成)
  → 逐 anchor 解码:类别取 argmax 过 confidence_threshold,
     5 个关键点 (x, y, conf) 做反算 x / scale_x + crop_x
  → 关键点 x/y < 0 或有效关键点 < min_valid_keypoints 则丢弃
  → 中心距 NMS(nms_distance_threshold),中心 = 有效关键点均值
  → refine():颜色差分二值轮廓 + 关键点语义 → 装甲板 / 灯臂 / 中心 R 的轮廓与可用性
  → Result{runes},每个 rune 带上 color
```

- **裁剪而非 letterbox**:模型在 4:3(1440×1080)零 padding 上训练,居中裁剪可让非 4:3 相机
  也不出现灰边;4:3 时等价于整帧缩放。
- **关键点顺序**固定为 `top, left, R, right, bottom`,索引 2 是 R 标(符心旋转中心)。
- **color** 由下发的 `VisionMode` 决定,写入每个结果。

## 接口

```cpp
struct DetectorConfig
{
    std::string model;                                  // onnx 路径,必填
    std::string device{"CPU"};                          // OpenVINO 设备
    float       confidence_threshold{0.8F};             // 类别置信度阈值
    float       keypoint_confidence_threshold{0.8F};    // 单关键点置信度阈值
    float       nms_distance_threshold{30.0F};          // 中心距 NMS 阈值(像素)
    int         min_valid_keypoints{3};                 // 有效关键点少于此数则丢弃
    RefinerConfig refiner;                              // 识别后精修参数
};

[[nodiscard]] DetectorConfig load_detector_config(const tools::config::Config &config);

class Detector
{
  public:
    explicit Detector(const DetectorConfig &config);
    struct Result { std::vector<Rune2d> runes; };
    [[nodiscard]] Result detect(const cv::Mat &bgr, Color color = Color::red);
};
```

- `Detector` 是 move-only(pimpl),不可拷贝。
- 构造时按文件名查 `find_model_spec`;未注册的模型抛 `unsupported rune model: <name>`。
- `Rune2d`:

```cpp
struct Rune2d
{
    Kind      kind{Kind::inactive};   // inactive / small_activated / big_activated
    Color     color{Color::red};      // red / blue(由 VisionMode 写入)
    float     confidence{0.0F};
    std::array<cv::Point2f, KEYPOINT_COUNT> keypoints{};           // top, left, R, right, bottom
    std::array<float, KEYPOINT_COUNT>       keypoint_confidence{};
    RuneRefinement                          refinement;            // 识别后精修
};
```

## 配置

`apps/infantry/config.yaml` 的 `buff` 段:

```yaml
buff:
  model: modules/auto_buff/models/shenzhenbuff-0624.onnx # 相对仓库根目录
  device: CPU
  confidence_threshold: 0.8
  keypoint_confidence_threshold: 0.8
  nms_distance_threshold: 30.0
  min_valid_keypoints: 3
  refine: # 识别后精修(深大 RP-26Rune 精简版)
    red_minus_blue_threshold: 60
    blue_minus_red_threshold: 62
    armor_module_area_relative_error_threshold: 0.35
    light_arm_line_samples: 30
    solidity_threshold_ellipse: 0.8
    solidity_threshold_rectangular: 0.66
    expect_aspect_ratio: 5.0
    aspect_ratio_relative_error_threshold: 0.42
    approx_error_tolerance: 0.01
    roi_margin_ratio: 0.15
    border_margin: 2.0
```

| 键 | 默认 | 说明 |
|---|---|---|
| `buff.model` | 无(必填) | onnx 路径,相对路径按仓库根解析 |
| `buff.device` | `CPU` | OpenVINO 设备字符串 |
| `buff.confidence_threshold` | `0.8` | 类别置信度下限 |
| `buff.keypoint_confidence_threshold` | `0.8` | 单个关键点置信度下限 |
| `buff.nms_distance_threshold` | `30.0` | 中心距 NMS 阈值(像素) |
| `buff.min_valid_keypoints` | `3` | 有效关键点少于该值则丢弃 |
| `buff.refine.*` | 见上 | 精修阈值,含义见「识别后精修」 |

## shenzhenbuff 模型输出布局

单输出,形状 `[1, 18, 6300]` 或 `[1, 6300, 18]`,**18 = 3 类 + 5 关键点 × 3**:

| 字段 | 数量 | 含义 |
|---|---|---|
| `class` | 3 | `inactive, small_activated, big_activated` |
| `keypoints` | 5 × 3 | 每个关键点 `x, y, conf`,顺序 `top, left, R, right, bottom` |

`6300` = 80×60 + 40×30 + 20×15(三个尺度的 anchor 之和)。解码在 `models/shenzhenbuff.cpp`。

## 识别后精修

移植自深大 RP-26Rune 的 `RuneObservationRefiner` 精简版:网络五点给「语义」,颜色差分二值轮廓给
「像素真相」,形状描述符给「可用性门控」。三步:

1. **取轮廓**:由 4 个靶角点(`top/left/right/bottom`)外接框外扩 `roi_margin_ratio` 得 ROI;
   颜色差分(红 `R-B`、蓝 `B-R`)→ 高斯模糊 → 阈值二值化 → `findContours`。
2. **分类**(三类互斥,命中多类则本帧丢弃):
   - *装甲板模块*:中心(`top/left/right/bottom` 均值)在轮廓内,且轮廓面积相对网络框面积
     (`0.25·|top-bottom|·|left-right|·π`)误差 ≤ `armor_module_area_relative_error_threshold`。
   - *灯臂*:中心与 `R` 点均在轮廓外,且直线 `中心→R` 穿过轮廓、直线 `left→right` 不穿过。
   - *中心 R 标*:`R` 在轮廓内,其余四点与中心均在轮廓外。
3. **可用性**:
   - 装甲板:椭圆描述符(solidity > `solidity_threshold_ellipse` 且不贴边);
   - 未激活灯臂:矩形描述符(solidity > `solidity_threshold_rectangular`,长宽比相对
     `expect_aspect_ratio` 误差 < `aspect_ratio_relative_error_threshold`);
   - 已激活灯臂:峰形描述符(`approxPolyDP` 拐点数,大符 8–16、小符 12–16,大符还须不贴边);
   - 中心 R:找到即可用。贴边定义为点到图像边界距离 ≤ `border_margin`。

结果写入 `Rune2d::refinement`(`RuneRefinement`):三类轮廓 + `is_armor_module_usable` /
`is_light_arm_usable` / `is_center_r_usable`。



## 扩展新模型

1. 新增 `models/<name>.hpp` / `.cpp`,给出通道数与 `decode` 函数;
2. 在 `models/model.cpp` 的 `find_model_spec` 里登记一行;
3. 在 yaml 里把 `buff.model` 指向新模型。
