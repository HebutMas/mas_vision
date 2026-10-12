# track 装甲板跟踪

单目标装甲板跟踪。把 `Solver` 解出的 3D 装甲板（云台系）输入，输出整车 11 维 EKF 状态（位置、速度、朝向、转速、几何），并支持纯外推给火控选板。

## 数据流

```
Detector::Result.armors          Armor2d(像素四角+兵种+颜色+置信度)
  → Solver::solve()              PnP: 2D 角点 → 云台系 Armor3d(x,y,z,yaw,kind,color,confidence)
  → Tracker::track()             过滤颜色 → 选目标 → 关联 → 状态机 → 健康监控
  → std::optional<TargetState>   整车状态,空 = 丢目标
```

世界系补偿由上层 `AutoAim::process(bgr, stamp, enemy_color, R_gimbal2world)` 完成：把观测旋到世界系再跟踪，输出旋回云台系。`r_gimbal2world` 省略（单位阵）时全程在云台系。

## 分层与文件职责

| 文件 | 层 | 职责 |
|---|---|---|
| `solver.hpp` / `solver.cpp` | **感知层** | 单板 PnP、相机内参与板材几何 |
| `target.hpp` / `target.cpp` | **模型层** | 单台车 11 维 EKF 与整车几何 |
| `tracker.hpp` / `tracker.cpp` | **策略层** | 选目标、关联、4 态状态机、配置加载 |

## 感知层 Solver

```cpp
struct ArmorGeometry // 毫米:width = 两灯条间距,height = 灯条发光长(四角即灯条端点)
{
    double small_width{135.0}, small_height{55.0};
    double large_width{225.0}, large_height{55.0};
};

struct SolverConfig { cv::Mat camera_matrix; cv::Mat distort_coeff; ArmorGeometry geometry; };

[[nodiscard]] SolverConfig load_solver_config(const tools::config::Config &config);

class Solver
{
  public:
    explicit Solver(SolverConfig config);
    [[nodiscard]] std::optional<Armor3d> solve(const Armor2d &armor) const;
};
```

- `cv::solvePnP(..., SOLVEPNP_IPPE)`；物点本体系 `x=法线(前), y=左, z=上`，顺序与 `corners` 的 `tl,tr,br,bl` 对齐。
- 相机系(OpenCV `x右,y下,z前`)→云台系(`x前,y左,z上`)：`(tx,ty,tz) → (tz, -tx, -ty)`。
- **法线二义性**：矩形板 IPPE 可能返回背面解（`yaw` 偏 π）。用“只可能看到正面”约束，`normal·t > 0` 时取反。
- 输出的 `Armor3d::yaw` 是**板位置方位角**（`center - r·(cos,sin)` 里的 θ），与 `TargetState::armor_yaw(id)` 同约定。

## 模型层 Target

状态 `x = [xc, vxc, yc, vyc, zc, vzc, yaw, w, r, l, h]`（位置/速度/朝向/转速/几何）。

板几何：第 `id` 块板位于

```
θ = yaw + id·2π/N,  r_i = r + (id 奇 ? l : 0),  h_i = (id 奇 ? h : 0)
位置 = (xc - r_i·cosθ, yc - r_i·sinθ, zc + h_i)
```

观测 `z = [方位 φ, 俯仰 p, 距离 d, 板朝向 ψ]`（`ψ = θ`），`H` 为解析雅可比（`xyz2ypd_jacobian` × 板位置对状态的偏导）。

| 方法 | 说明 |
|---|---|
| `predict(dt)` | 恒速 F；白噪声加速度 Q。按 `\|w\|` 分三档选 `v1/v2`（`slow`/`normal`/`fast`，带 ±0.5 滞回）；`r/l/h` 过程噪声 `q_rlh·dt`；预测后包裹 `x[6]`；前哨站 `\|w\|>2` 钳到 `outpost_w` |
| `match(obs, gate)` | 对每个 `id` 算完整 4 维新息马氏距离 `d²=eᵀS⁻¹e`，取最小者；`d² ≥ gate` 返回 `nullopt`。位置项自动打破 4 板 180° 二义性 |
| `update(obs, id)` | Joseph 形式协方差更新；`e[0]/e[3]` 做角度包裹 |
| `recover(obs, id)` | 自恢复：中心重锚到观测板、`r` 回 `radius`、`l=h=0`、速度清零、`P = p0`，**保留转速 `w`** |
| `unhealthy(ratio)` | 发散（`r∉(0.05,0.7)` ∨ `r+l∉(0.07,0.9)` ∨ `\|w\|>50`）或 NIS 失败比 > `ratio` |
| `state(jumped)` | 打包成 `TargetState`（附最近匹配观测的 `kind/color/confidence`） |

`ModelParams` 里的三档噪声默认值：`slow`(`|w|<2`) `v1=10, v2=0.5`；`normal` `v1=100, v2=400`；`fast`(`|w|≥8`) `v1=500, v2=1000`。

### 输出给火控

```cpp
[[nodiscard]] TargetState                  predict_at(const TargetState &s, double t_future);   // 纯外推,不动滤波器
[[nodiscard]] std::vector<Eigen::Vector3d> armor_positions_at(const TargetState &s, double t_future);
```

`predict_at` 按恒速/恒转速外推 `center` 与 `yaw`，`armor_positions_at` 给出该时刻 4 块板的世界位置——供弹道侧迭代飞行时间后选板。

## 策略层 Tracker

```cpp
class Tracker
{
  public:
    explicit Tracker(TrackerConfig config = {}, std::optional<Color> enemy_color = std::nullopt);
    std::optional<TargetState> track(const std::vector<Armor3d> &armors, double stamp);
};
```

状态机（`State` 私有）：

```
lost ──有候选──> detecting ──连续命中≥min_detect_count──> tracking
  ^                  │                                      │
  │                  └──任一帧无候选──> lost                └──无命中──> temp_lost
  │                                                                          │
  └───────────连续丢失 > max_temp_lost_count(前哨站 75)──────────────────────┘
                                        temp_lost ──重新命中──> tracking
```

`track(armors, stamp)` 的流水线：

1. 算 `dt`（首帧 0），`dt > dt_max` 视为掉线 → `lost`；
2. 按 `enemy_color` 过滤候选；
3. `lost`：有候选则 `select()` 选一块建 `Target` 进入 `detecting`；
4. 否则 `predict(dt)` → `associate()` → `update()` → `update_state()` → `try_switch()`；
5. 健康监控：`unhealthy` 且有观测则**原地 `recover`**，无观测才转 `lost`；
6. 返回 `TargetState`，`tracking = (tracking ∨ temp_lost)`。

- **选目标** `select()`：到光学中心（图像中心）角距最近者优先，同距取优先级高（数字小）者。
- **关联** `associate()`：只接受 `armor_num` 相同的 `Kind` 组，再逐个过马氏门限，取 `maha2` 最小者。
- **跳板**：`id` 变化且 `|e_yaw| > jump_yaw_diff` 时只置 `jumped = true`
- **切换**：`tracking/temp_lost` 下出现更高优先级目标，连续 `switch_confirm_count` 帧才重初始化。

## 配置

`apps/infantry/config.yaml` 的 `auto_aim.track` 段：

```yaml
track:
  tracker:
    min_detect_count: 5
    max_temp_lost_count: 25
    outpost_max_temp_lost_count: 75
    switch_confirm_count: 3
    max_match_mahalanobis2: 50.0
    nis_reset_ratio: 0.4
    jump_yaw_diff: 0.4
    dt_max: 0.1
  ekf:
    v1: 100.0
    v2: 400.0
    slow: {w_thres: 2.0, v1: 10.0, v2: 0.5}
    fast: {w_thres: 8.0, v1: 100.0, v2: 1000.0}
    q_rlh: 1.0e-4
    r_yaw: 4.0e-3
    r_pitch: 4.0e-3
    r_dist: 9.0e-2
    r_armor_yaw: 9.0e-2
    default_radius: 0.26
    outpost_radius: 0.2765
    base_radius: 0.3205
    outpost_v2: 0.1
    outpost_w_clamp: 2.51
    p0: [1.0, 64.0, 1.0, 64.0, 1.0, 64.0, 0.4, 10.0, 0.04, 0.04, 0.04]
```

| 键 | 说明 |
|---|---|
| `tracker.min_detect_count` | `detecting → tracking` 需要的连续命中帧数 |
| `tracker.max_temp_lost_count` / `outpost_max_temp_lost_count` | `temp_lost` 容忍的最大连续丢失帧数 |
| `tracker.switch_confirm_count` | 更高优先级目标连续确认帧数才切换 |
| `tracker.max_match_mahalanobis2` | 关联马氏距离平方门限（`χ²(4)` 95% ≈ 9.49，放宽到 50） |
| `tracker.nis_reset_ratio` | NIS 滑窗失败比超过它判 `unhealthy` |
| `tracker.jump_yaw_diff` | 判定跳板的板朝向残差阈值（rad） |
| `tracker.dt_max` | 帧间隔上限（s），超过判掉线 |
| `ekf.v1/v2` | `normal` 档平移/角加速度方差 |
| `ekf.slow.{w_thres,v1,v2}` / `ekf.fast.{w_thres,v1,v2}` | 慢转/快转档 |
| `ekf.q_rlh` | `r/l/h` 过程噪声系数 |
| `ekf.r_yaw/r_pitch/r_dist/r_armor_yaw` | 观测噪声对角（距离/板朝向项随量测自适应） |
| `ekf.default_radius` / `outpost_radius` / `base_radius` | 各车型半径初值 |
| `ekf.outpost_v2` / `outpost_w_clamp` | 前哨站恒转速先验的角加速度方差与转速钳位 |
| `ekf.p0` | 初始协方差对角，顺序 `x,vx,y,vy,z,vz,yaw,w,r,l,h`；长度不符则忽略 |

## 已知限制

- `r/l/h`（几何）弱可观：`r` 与 `center` 沿板方位方向退化，只在多板/运动下才收敛，静止单板时靠 `p0` 约束。
- 远距离板法线病态时 `yaw` 观测噪声大，`w` 会有小量摆动。
- 单目标：同一时刻只跟踪一台车，更高优先级目标需连续确认才切换。
