#pragma once

#include "hardware/serialport/serialtypes.hpp"
#include "modules/auto_armor/detection/detector.hpp"
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
    explicit Debug(const std::string &application_id);
    ~Debug();

    Debug(const Debug &)            = delete;
    Debug &operator=(const Debug &) = delete;

    // Viewer 是否处于活动状态。
    [[nodiscard]] bool active() const;

    // 推入一帧识别结果(含当前云台串口数据)。
    void push(const cv::Mat &image, std::int64_t frame, tools::time::TimePoint timestamp, const hardware::serialport::ReceivePacket &serial,
              rm::armor::Detector::Result detection);

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
        rm::armor::Detector::Result         detection;
    };

    tools::LatestFrame<DebugFrame> debug_in_;
    std::thread                    debug_thread_;
#endif
};

} // namespace infantry
