#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "modules/auto_armor/track/target.hpp"
#include "tools/config/config.hpp"

// 策略层:单目标的选目标 + 关联 + 状态机
namespace rm::armor::track
{
// 状态机/关联参数
struct TrackerParams
{
    int    min_detect_count{5};
    int    max_temp_lost_count{25};
    int    outpost_max_temp_lost_count{75};
    int    switch_confirm_count{3};
    double max_match_mahalanobis2{50.0};
    double nis_reset_ratio{0.4};
    double jump_yaw_diff{0.4}; // rad,判定跳板的板朝向残差阈值
    double dt_max{0.1};        // s,掉线 guard
};

// 车型策略，装甲板块数与优先级,按 Kind 索引。
struct KindPolicy
{
    // Kind 顺序 hero,engineer,infantry1,infantry2,infantry3,sentry,outpost,base
    std::array<int, 8> armor_num{4, 4, 4, 4, 4, 4, 3, 3};
    std::array<int, 8> priority{1, 2, 3, 3, 3, 4, 5, 6}; // 数字小者优先

    [[nodiscard]] int armor_num_of(Kind kind) const { return armor_num[static_cast<std::size_t>(kind)]; }
    [[nodiscard]] int priority_of(Kind kind) const { return priority[static_cast<std::size_t>(kind)]; }
};

// track 总配置
struct TrackerConfig
{
    TrackerParams tracker;
    ModelParams   ekf;
    KindPolicy    kind;

    double outpost_radius{0.2765};
    double base_radius{0.3205};
    double outpost_v2{0.1};       // 前哨/基地恒转速先验角加速度
    double outpost_w_clamp{2.51}; // 前哨站收敛后 |w| 钳位

    [[nodiscard]] ModelParams model_params_for(Kind kind) const;
};

// 从 yaml 读取
[[nodiscard]] TrackerConfig load_tracker_config(const tools::config::Config &config);

// 单目标跟踪器
class Tracker
{
  public:
    explicit Tracker(TrackerConfig config = {}, std::optional<Color> enemy_color = std::nullopt);

    std::optional<TargetState> track(const std::vector<Armor3d> &armors, double stamp);

  private:
    enum class State : std::uint8_t
    {
        lost,
        detecting,
        tracking,
        temp_lost,
    };

    struct Association
    {
        const Armor3d *armor;
        Target::Match  match;
    };

    // 用首个候选新建目标
    void begin(const Armor3d &pick);

    // 状态机推进
    void update_state(bool matched);

    // 更高优先级目标连续确认后重初始化;返回是否发生切换
    bool try_switch(const std::vector<Armor3d> &candidates);

    [[nodiscard]] std::optional<Association> associate(const std::vector<Armor3d> &candidates) const;
    [[nodiscard]] const Armor3d             &select(const std::vector<Armor3d> &candidates) const;

    TrackerConfig           config_;
    std::optional<Color>    enemy_color_;
    std::unique_ptr<Target> target_;
    State                   state_{State::lost};
    double                  last_stamp_{0.0};
    bool                    initialized_{false};
    int                     hit_{0};
    int                     lost_n_{0};
    int                     switch_n_{0};
};

} // namespace rm::armor::track
