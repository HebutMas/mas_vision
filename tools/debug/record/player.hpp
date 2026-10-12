#pragma once

#include "tools/time/time.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace tools::record
{

// 回放配置。
struct PlayerConfig
{
    std::string path;           // 视频文件路径
    double      fps{0.0};       // 播放帧率;<=0 用视频自带帧率(取不到时按 30)
    bool        realtime{true}; // 按帧率实时出帧;false 则尽全力跑
};

// 视频文件循环回放,解码在后台线程进行
class Player
{
  public:
    explicit Player(PlayerConfig config);
    ~Player();
    Player(const Player &)            = delete;
    Player &operator=(const Player &) = delete;

    // 视频是否已打开。
    [[nodiscard]] bool is_open() const;

    // 取下一帧。后台线程解不出帧(文件损坏/重开失败)或对象已析构时返回 false。
    bool read(cv::Mat &image, tools::time::TimePoint &timestamp);

  private:
    struct Frame
    {
        cv::Mat                image;
        tools::time::TimePoint timestamp;
    };

    void run(); // 后台解码线程

    PlayerConfig     config_;
    cv::VideoCapture capture_;
    double           fps_{30.0};
    bool             opened_{false};

    std::thread             worker_;
    std::mutex              mutex_;
    std::condition_variable cv_;
    std::optional<Frame>    ready_; // 已解码、待调用方取走
    bool                    stopped_{false};
    bool                    failed_{false};
};

} // namespace tools::record
