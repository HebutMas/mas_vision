#pragma once

#include "hardware/serialport/serialtypes.hpp"
#include "modules/auto_armor/auto_armor.hpp"
#include "modules/auto_buff/detection/detector.hpp"
#include "tools/config/config.hpp"
#include "tools/debug/debug.hpp"
#include "tools/time/time.hpp"

#include <opencv2/core.hpp>

#include <cstdint>
#include <string>

#ifdef RM_DEBUG
#include "tools/latest_frame/latest_frame.hpp"

#include <thread>
#endif

namespace infantry
{

// 远程调试封装
class Debug
{
  public:
    // application_id 用于在 Viewer 里区分不同程序,建议用 "rm_vision.<兵种>"。
    Debug(const std::string &application_id, const tools::config::Config &config);
    ~Debug();

    Debug(const Debug &)            = delete;
    Debug &operator=(const Debug &) = delete;

    // Viewer 是否处于活动状态。
    [[nodiscard]] bool active() const;

    // 推入一帧识别结果(含当前云台串口数据与本次推理耗时 ms)。
    void push(const cv::Mat &image, std::int64_t frame, tools::time::TimePoint timestamp, const hardware::serialport::ReceivePacket &serial,
              rm::armor::AutoAimResult result, double latency_ms);
    void push(const cv::Mat &image, std::int64_t frame, tools::time::TimePoint timestamp, const hardware::serialport::ReceivePacket &serial,
              rm::buff::Detector::Result detection, double latency_ms);

  private:
    tools::debug::Sink sink_;

#ifdef RM_DEBUG
    // 推给 debug 线程的一帧快照。
    struct DebugFrame
    {
        cv::Mat                             image;
        std::int64_t                        frame{};
        tools::time::TimePoint              timestamp;
        hardware::serialport::ReceivePacket serial;
        rm::armor::AutoAimResult            armor_result;
        double                              latency_ms{};
        rm::buff::Detector::Result          rune_detection;
    };

    tools::LatestFrame<DebugFrame> debug_in_;
    std::thread                    debug_thread_;
#endif
};

} // namespace infantry
