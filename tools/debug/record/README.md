# record 录像

把相机帧编码成 H.264 落盘,并在**同名 CSV** 里逐帧记录时间戳与附带数据(如串口姿态),用于离线复盘 / 与其它日志对齐。

复用 [`video_encoder.hpp`](../video/video_encoder.hpp)

## 输出

录制一段会在 `config.directories` 中第一个可用目录下生成两个文件(名字相同,后缀不同):

```
autoaim_2026-10-01_20-51-29.h264   # Annex B H.264 码流,可直接播放 / 送 ffmpeg
autoaim_2026-10-01_20-51-29.csv    # 每帧一行
```

CSV 列:`t_ns` + `config.columns`。`t_ns` 是相对 [`tools::time::base()`](../../time/time.hpp) 的纳秒数,和调试链路用的是同一个时钟,可直接相减对齐。例如默认列:

```csv
t_ns,qw,qx,qy,qz
371,1,0,0,0
16000371,1,0,0,0
```

## 接口

```cpp
namespace tools::record {

struct RecordConfig {
    std::vector<std::string> directories{"record", "/tmp/autoaim"}; // 依次尝试,取第一个可用
    std::vector<std::string> columns{"qw", "qx", "qy", "qz"};      // CSV 数据列名
    std::chrono::seconds max_duration{60};        // 单段最长时长,超时自动停止并保存
    std::uintmax_t max_total_size{30ULL * 1024 * 1024 * 1024}; // 目录内已有录像体积上限
    std::uintmax_t min_free_space{50ULL * 1024 * 1024 * 1024}; // 开始录制所需最小剩余空间
    int            fps{200};                      // 编码时间基 / gop 参考帧率
    std::int64_t   bitrate{8'000'000};            // 目标码率 bit/s
    int            gop{200};                      // 关键帧间隔(帧)
};

class Recorder {
  public:
    explicit Recorder(RecordConfig config = {});
    [[nodiscard]] bool start();                   // 失败返回 false,原因见 status()
    void stop(bool save = true);                  // save=false 丢弃已写文件
    [[nodiscard]] bool recording() const;
    [[nodiscard]] std::optional<std::string> filename() const;  // 未录制时为空
    [[nodiscard]] std::string status() const;     // 录制中 / 最近失败或自动停止原因
    void push(const cv::Mat &bgr, tools::time::TimePoint timestamp, const std::vector<double> &values = {});
};

} // namespace tools::record
```

- 帧尺寸由**首帧**决定(编码器在首帧按 `cols x rows` 构造)。
- `push()` 当帧编码并写文件,**在调用线程串行执行**。
- `start()` 会检查:目录可用性、剩余空间、目录内已有录像总量。
- 超过 `max_duration` 时下一次 `push()` 自动停止并保存。
- 析构时若仍在录制,会自动 `stop(true)` 收尾。

## 日志

状态都通过 `tools::debug::log(..., "record")` 输出,控制台 + Rerun 双写:

- `record started: <h264>` —— 开始录制(文件已建好)
- `record video: <h264> WxH (VAAPI|CPU)` —— 首帧到达,编码器就绪
- `record stopped: <h264> saved (N bytes)` / `... discarded` —— 结束录制(保存 / 丢弃)
- `record start failed: <原因>` —— 启动失败(目录不可用、空间不足、超限、建不了文件、编码器建不起来)
- `record failed: encode: <原因>` / `record failed: flush: <原因>` —— 录制中出错

## 构建与测试

```bash
cmake --build build -j4
ctest --test-dir build -R record --output-on-failure
./build/tools/record_test
```

## 接入 app

仿照调试线程:构造一个 `Recorder`,`start()` 后用主循环里的帧 + 串口姿态喂进去。

```cpp
tools::record::Recorder recorder;   // 默认普通调试列 qw/qx/qy/qz
if (!recorder.start()) { /* 看 recorder.status() */ }

// 主循环里:
recorder.push(frame.image, frame.timestamp, {state.qw, state.qx, state.qy, state.qz});
```

记录其它数据时改 `config.columns` 并把对应数值按顺序放进 `push()` 的 `values` 即可。
