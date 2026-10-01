#pragma once

#include "tools/time/time.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace tools::record
{

// 录制配置。
struct RecordConfig
{
    // 依次尝试的保存目录,第一个可用的被选中。
    std::vector<std::string> directories{"record", "/tmp/autoaim"};
    // CSV 数据列名(时间戳列自动加在首位),顺序与 push() 的 values 一一对应。
    std::vector<std::string> columns{"qw", "qx", "qy", "qz"};
    std::chrono::seconds     max_duration{60};                           // 单段最长时长,超时自动停止并保存
    std::uintmax_t           max_total_size{30ULL * 1024 * 1024 * 1024}; // 目录内已有录像的体积上限
    std::uintmax_t           min_free_space{50ULL * 1024 * 1024 * 1024}; // 开始录制所需的最小剩余空间
    int                      fps{200};                                   // 编码时间基 / 关键帧间隔的参考帧率
    std::int64_t             bitrate{8'000'000};                         // 目标码率 bit/s
    int                      gop{200};                                   // 关键帧间隔(帧)
};

// 把相机帧编码成 H.264 落盘,并在同名 CSV 里逐帧记录时间戳与附带数据
class Recorder
{
  public:
    explicit Recorder(RecordConfig config = {});
    ~Recorder();
    Recorder(const Recorder &)            = delete;
    Recorder &operator=(const Recorder &) = delete;

    // 开始一段录制。目录不可用 / 空间不足 / 已有录像超限时返回 false,原因见 status()。
    [[nodiscard]] bool start();

    // 停止录制;save=false 时丢弃已写文件。
    void stop(bool save = true);

    [[nodiscard]] bool recording() const;

    // 当前视频文件路径(未录制时为空)。
    [[nodiscard]] std::optional<std::string> filename() const;

    // 当前状态:录制中 / 最近一次失败或自动停止的原因。
    [[nodiscard]] std::string status() const;

    // 推入一帧:编码写 H.264,并写一行 CSV(时间戳 + values,逐列对应 config.columns)。
    void push(const cv::Mat &bgr, tools::time::TimePoint timestamp, const std::vector<double> &values = {});

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace tools::record
