#include "tools/debug/record/player.hpp"

#include "tools/debug/debug.hpp"

#include <chrono>
#include <utility>

namespace tools::record
{

Player::Player(PlayerConfig config) : config_(std::move(config))
{
    opened_ = capture_.open(config_.path);
    fps_    = config_.fps > 0.0 ? config_.fps : capture_.get(cv::CAP_PROP_FPS);
    if (fps_ <= 0.0)
    {
        fps_ = 30.0;
    }
    if (!opened_)
    {
        failed_ = true;
    }
    else
    {
        worker_ = std::thread([this] { run(); });
    }
}

Player::~Player()
{
    {
        std::scoped_lock const lock(mutex_);
        stopped_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable())
    {
        worker_.join();
    }
}

bool Player::is_open() const { return opened_; }

void Player::run()
{
    std::int64_t           frame = 0;
    tools::time::TimePoint play_start;
    while (true)
    {
        cv::Mat image;
        if (!capture_.read(image) || image.empty())
        {
            // 循环播放:读完重开回到开头
            capture_.open(config_.path);
            if (!capture_.isOpened() || !capture_.read(image) || image.empty())
            {
                std::scoped_lock const lock(mutex_);
                failed_ = true;
                cv_.notify_all();
                return;
            }
            frame = 0;
            tools::debug::log(tools::debug::Level::info, "video loop restart: " + config_.path);
        }
        if (frame == 0)
        {
            // 首帧才开始计时,避免构造与实际播放之间的空档被计入。
            play_start = tools::time::Clock::now();
        }
        ++frame;
        if (config_.realtime)
        {
            // 目标时刻已过则立即返回,不必额外判断。
            std::this_thread::sleep_until(play_start + std::chrono::duration_cast<tools::time::Clock::duration>(
                                                           std::chrono::duration<double>(static_cast<double>(frame) / fps_)));
        }
        // 出流时刻,帧就绪的时刻
        const tools::time::TimePoint timestamp = tools::time::Clock::now();

        // 交接给调用方;上一帧还没被取走就等,保证不丢帧。
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return !ready_.has_value() || stopped_; });
        if (stopped_)
        {
            return;
        }
        ready_ = Frame{std::move(image), timestamp};
        lock.unlock();
        cv_.notify_all();
    }
}

bool Player::read(cv::Mat &image, tools::time::TimePoint &timestamp)
{
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return ready_.has_value() || stopped_ || failed_; });
    if (stopped_ || !ready_)
    {
        return false;
    }
    image     = std::move(ready_->image);
    timestamp = ready_->timestamp;
    ready_.reset();
    lock.unlock();
    cv_.notify_all();
    return true;
}

} // namespace tools::record
