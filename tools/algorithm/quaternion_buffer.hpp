#pragma once

#include "tools/time/time.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <mutex>

namespace tools
{

// 定长环形缓冲:按到达时刻存放 IMU 四元数,容量按「采样率 × 覆盖时长」取。
template <std::size_t Capacity> class TimedQuaternionBuffer
{
    static_assert(Capacity >= 2, "The capacity must be at least 2 in order to perform interpolation.");

  public:
    // 采集线程写入一个样本;零 / 未归一化输入会被归一化后存储。
    void push(tools::time::TimePoint time, const Eigen::Quaternionf &q)
    {
        const float                 norm = q.norm();
        const Eigen::Quaternionf    unit = norm > 1e-6F ? q.normalized() : Eigen::Quaternionf::Identity();
        std::lock_guard<std::mutex> lock(mutex_);
        samples_[head_] = Sample{time, unit};
        head_           = (head_ + 1) % Capacity;
        size_           = std::min(size_ + 1, Capacity);
    }

    // 清空全部样本
    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        head_ = 0;
        size_ = 0;
    }

    // 查询 time 时刻姿态:夹在两侧样本之间用 Eigen slerp(最短弧)插值;
    // 超出范围取最近端点(不外推);无样本返回单位四元数。
    [[nodiscard]] Eigen::Quaternionf at(tools::time::TimePoint time) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (size_ == 0)
        {
            return Eigen::Quaternionf::Identity();
        }
        const std::size_t newest = (head_ + Capacity - 1) % Capacity;
        if (time >= samples_[newest].time)
        {
            return samples_[newest].q;
        }
        // 从最新往回找第一个不晚于 time 的样本,与其后继构成插值区间。
        std::size_t newer = newest;
        for (std::size_t i = 0; i + 1 < size_; ++i)
        {
            const std::size_t older = (newer + Capacity - 1) % Capacity;
            if (samples_[older].time <= time)
            {
                const float u = seconds(time - samples_[older].time) / seconds(samples_[newer].time - samples_[older].time);
                return samples_[older].q.slerp(u, samples_[newer].q);
            }
            newer = older;
        }
        // time 早于最旧样本:取最旧端点。
        const std::size_t oldest = (head_ + Capacity - size_) % Capacity;
        return samples_[oldest].q;
    }

  private:
    struct Sample
    {
        tools::time::TimePoint time;
        Eigen::Quaternionf     q;
    };

    template <typename Rep, typename Period> static float seconds(const std::chrono::duration<Rep, Period> &span)
    {
        return std::chrono::duration<float>(span).count();
    }

    mutable std::mutex           mutex_;
    std::array<Sample, Capacity> samples_{};
    std::size_t                  head_{0}; // 下一个写入位置。
    std::size_t                  size_{0}; // 有效样本数(<= Capacity)。
};

} // namespace tools
