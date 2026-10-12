#include "modules/auto_armor/track/tracker.hpp"

#include <cmath>
#include <limits>
#include <utility>

#include "tools/algorithm/math_tools.hpp"

namespace rm::armor::track
{

ModelParams TrackerConfig::model_params_for(Kind k) const
{
    ModelParams p = ekf;
    p.armor_num   = kind.armor_num_of(k);
    switch (k)
    {
    case Kind::outpost: // 固定半径 + 恒转速,收敛后钳 w
        p.radius          = outpost_radius;
        p.q_rlh           = 1.0e-6;
        p.v2              = outpost_v2;
        p.p0[8]           = 1.0e-4;
        p.outpost_w_clamp = true;
        p.outpost_w       = outpost_w_clamp;
        break;
    case Kind::base: // 按前哨处理但不钳 w
        p.radius = base_radius;
        p.q_rlh  = 1.0e-6;
        p.v2     = outpost_v2;
        p.p0[8]  = 1.0e-4;
        break;
    default:
        break;
    }
    return p;
}

TrackerConfig load_tracker_config(const tools::config::Config &config)
{
    TrackerConfig     c; // 默认值起步:缺失键不报错,只覆盖提供的项
    const std::string t = "auto_aim.track.";

    c.tracker.min_detect_count            = config.value(t + "tracker.min_detect_count", c.tracker.min_detect_count);
    c.tracker.max_temp_lost_count         = config.value(t + "tracker.max_temp_lost_count", c.tracker.max_temp_lost_count);
    c.tracker.outpost_max_temp_lost_count = config.value(t + "tracker.outpost_max_temp_lost_count", c.tracker.outpost_max_temp_lost_count);
    c.tracker.switch_confirm_count        = config.value(t + "tracker.switch_confirm_count", c.tracker.switch_confirm_count);
    c.tracker.max_match_mahalanobis2      = config.value(t + "tracker.max_match_mahalanobis2", c.tracker.max_match_mahalanobis2);
    c.tracker.nis_reset_ratio             = config.value(t + "tracker.nis_reset_ratio", c.tracker.nis_reset_ratio);
    c.tracker.jump_yaw_diff               = config.value(t + "tracker.jump_yaw_diff", c.tracker.jump_yaw_diff);
    c.tracker.dt_max                      = config.value(t + "tracker.dt_max", c.tracker.dt_max);

    c.ekf.v1          = config.value(t + "ekf.v1", c.ekf.v1);
    c.ekf.v2          = config.value(t + "ekf.v2", c.ekf.v2);
    c.ekf.slow_w      = config.value(t + "ekf.slow.w_thres", c.ekf.slow_w);
    c.ekf.slow_v1     = config.value(t + "ekf.slow.v1", c.ekf.slow_v1);
    c.ekf.slow_v2     = config.value(t + "ekf.slow.v2", c.ekf.slow_v2);
    c.ekf.fast_w      = config.value(t + "ekf.fast.w_thres", c.ekf.fast_w);
    c.ekf.fast_v1     = config.value(t + "ekf.fast.v1", c.ekf.fast_v1);
    c.ekf.fast_v2     = config.value(t + "ekf.fast.v2", c.ekf.fast_v2);
    c.ekf.q_rlh       = config.value(t + "ekf.q_rlh", c.ekf.q_rlh);
    c.ekf.r_yaw       = config.value(t + "ekf.r_yaw", c.ekf.r_yaw);
    c.ekf.r_pitch     = config.value(t + "ekf.r_pitch", c.ekf.r_pitch);
    c.ekf.r_dist      = config.value(t + "ekf.r_dist", c.ekf.r_dist);
    c.ekf.r_armor_yaw = config.value(t + "ekf.r_armor_yaw", c.ekf.r_armor_yaw);
    c.ekf.radius      = config.value(t + "ekf.default_radius", c.ekf.radius);
    c.outpost_radius  = config.value(t + "ekf.outpost_radius", c.outpost_radius);
    c.base_radius     = config.value(t + "ekf.base_radius", c.base_radius);
    c.outpost_v2      = config.value(t + "ekf.outpost_v2", c.outpost_v2);
    c.outpost_w_clamp = config.value(t + "ekf.outpost_w_clamp", c.outpost_w_clamp);

    const auto p0 = config.value<std::vector<double>>(t + "ekf.p0", {});
    if (p0.size() == static_cast<std::size_t>(c.ekf.p0.size()))
    {
        for (Eigen::Index i = 0; i < c.ekf.p0.size(); ++i)
        {
            c.ekf.p0[i] = p0[static_cast<std::size_t>(i)];
        }
    }

    return c;
}

Tracker::Tracker(TrackerConfig config, std::optional<Color> enemy_color) : config_(std::move(config)), enemy_color_(enemy_color) {}

void Tracker::begin(const Armor3d &pick)
{
    target_   = std::make_unique<Target>(pick, config_.model_params_for(pick.kind));
    state_    = State::detecting;
    hit_      = 1;
    lost_n_   = 0;
    switch_n_ = 0;
}

const Armor3d &Tracker::select(const std::vector<Armor3d> &candidates) const
{
    // 离图像中心最近者优先,同距离取优先级高(数字小)者
    const Armor3d *best      = &candidates.front();
    double         best_dist = std::numeric_limits<double>::infinity();
    int            best_prio = std::numeric_limits<int>::max();
    for (const auto &a : candidates)
    {
        const Eigen::Vector3d ypd  = tools::xyz2ypd(a.xyz);
        const double          dist = std::hypot(ypd[0], ypd[1]);
        const int             prio = config_.kind.priority_of(a.kind);
        if (dist < best_dist - 1e-9 || (std::fabs(dist - best_dist) <= 1e-9 && prio < best_prio))
        {
            best_dist = dist;
            best_prio = prio;
            best      = &a;
        }
    }
    return *best;
}

std::optional<Tracker::Association> Tracker::associate(const std::vector<Armor3d> &candidates) const
{
    if (!target_) return std::nullopt;
    // 避免因 Kind 抖动而漏关联(→temp_lost→lost)。几何仍由马氏门限约束。
    const int                  target_group = config_.kind.armor_num_of(target_->kind());
    std::optional<Association> best;
    for (const auto &a : candidates)
    {
        if (config_.kind.armor_num_of(a.kind) != target_group) continue;
        const auto m = target_->match(a, config_.tracker.max_match_mahalanobis2);
        if (m && (!best || m->maha2 < best->match.maha2)) best = Association{&a, *m};
    }
    return best;
}

void Tracker::update_state(bool matched)
{
    switch (state_)
    {
    case State::detecting:
        if (matched)
        {
            if (++hit_ >= config_.tracker.min_detect_count) state_ = State::tracking;
        }
        else
        {
            state_ = State::lost;
        }
        break;
    case State::tracking:
        if (!matched)
        {
            state_  = State::temp_lost;
            lost_n_ = 1;
        }
        break;
    case State::temp_lost:
        if (matched)
        {
            state_ = State::tracking;
        }
        else
        {
            const int limit = target_->kind() == Kind::outpost ? config_.tracker.outpost_max_temp_lost_count : config_.tracker.max_temp_lost_count;
            if (++lost_n_ > limit) state_ = State::lost;
        }
        break;
    case State::lost:
        break;
    }
}

bool Tracker::try_switch(const std::vector<Armor3d> &candidates)
{
    if (state_ != State::tracking && state_ != State::temp_lost) return false;

    // 更高优先级目标连续 switch_confirm_count 帧出现才重初始化
    const Armor3d *better    = nullptr;
    int            best_prio = config_.kind.priority_of(target_->kind());
    for (const auto &a : candidates)
    {
        if (a.kind != target_->kind() && config_.kind.priority_of(a.kind) < best_prio)
        {
            best_prio = config_.kind.priority_of(a.kind);
            better    = &a;
        }
    }
    if (better == nullptr)
    {
        switch_n_ = 0;
        return false;
    }
    if (++switch_n_ < config_.tracker.switch_confirm_count) return false;

    begin(*better);
    return true;
}

std::optional<TargetState> Tracker::track(const std::vector<Armor3d> &armors, double stamp)
{
    const double dt = initialized_ ? stamp - last_stamp_ : 0.0;
    last_stamp_     = stamp;
    initialized_    = true;
    if (dt > config_.tracker.dt_max) state_ = State::lost; // 掉线/掉帧 guard

    std::vector<Armor3d> candidates;
    candidates.reserve(armors.size());
    for (const auto &a : armors)
    {
        if (!enemy_color_ || a.color == *enemy_color_) candidates.push_back(a);
    }

    const Armor3d *matched_obs = nullptr;
    int            matched_id  = 0;
    bool           jumped      = false;

    if (state_ == State::lost)
    {
        if (candidates.empty()) return std::nullopt;
        begin(select(candidates));
    }
    else
    {
        target_->predict(dt);

        const auto assoc = associate(candidates);
        if (assoc)
        {
            matched_obs = assoc->armor;
            matched_id  = assoc->match.id;
            // 换板只置 jumped 标志,不硬写 yaw:由 EKF 用观测自行收敛
            jumped = (matched_id != target_->last_id() && std::fabs(assoc->match.e_yaw) > config_.tracker.jump_yaw_diff);
            target_->update(*matched_obs, matched_id);
        }

        update_state(assoc.has_value());
        if (try_switch(candidates)) jumped = false;

        // 健康监控触发则重初始化,有观测则原地重置 EKF(不丢帧,继续 tracking);否则丢目标。
        if (state_ != State::lost && target_->unhealthy(config_.tracker.nis_reset_ratio))
        {
            if (matched_obs != nullptr)
            {
                target_->recover(*matched_obs, matched_id);
            }
            else
            {
                state_ = State::lost;
            }
        }
    }
    if (state_ == State::lost) return std::nullopt;

    TargetState s = target_->state(jumped);
    s.tracking    = (state_ == State::tracking || state_ == State::temp_lost);
    return s;
}

} // namespace rm::armor::track
