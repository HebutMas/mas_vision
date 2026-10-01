#include "tools/debug/record/record.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace
{

namespace fs = std::filesystem;

int failures = 0;

void check(bool ok, const std::string &what)
{
    if (!ok)
    {
        std::cerr << "FAIL: " << what << "\n";
        ++failures;
    }
}

// 每帧不同亮度,便于人工确认画面在动。
cv::Mat make_frame(int index) { return {240, 320, CV_8UC3, cv::Scalar(index * 40 % 256, 64, 192)}; }

std::vector<std::string> read_lines(const fs::path &path)
{
    std::vector<std::string> lines;
    std::ifstream            in(path);
    std::string              line;
    while (std::getline(in, line))
    {
        lines.push_back(line);
    }
    return lines;
}

constexpr int FRAMES = 5;

} // namespace

int main()
{
    const fs::path dir = fs::temp_directory_path() / "rm_record_test";
    fs::remove_all(dir);
    fs::create_directories(dir);

    tools::record::RecordConfig config;
    config.directories    = {dir.string()};
    config.columns        = {"qw", "qx", "qy", "qz"};
    config.max_duration   = std::chrono::seconds(600); // 测试里不触发自动停止
    config.min_free_space = 0;                         // 临时目录在 tmpfs 上,别卡空间检查

    tools::record::Recorder recorder(config);
    check(!recorder.recording(), "starts idle");

    check(recorder.start(), "start returns true");
    check(recorder.recording(), "recording after start");
    check(recorder.status().rfind("recording", 0) == 0, "status reports recording");

    const tools::time::TimePoint begin = tools::time::now();
    for (int index = 0; index < FRAMES; ++index)
    {
        recorder.push(make_frame(index), begin + std::chrono::milliseconds(index * 5), {1.0, 0.5, 0.25, 0.125});
    }

    const std::optional<std::string> name = recorder.filename();
    check(name.has_value(), "filename available while recording");

    recorder.stop(true);
    check(!recorder.recording(), "stopped");

    if (name.has_value())
    {
        const fs::path video = *name;
        const fs::path csv   = fs::path(*name).replace_extension(".csv");

        check(fs::exists(video), "video file exists: " + *name);
        check(fs::file_size(video) > 0, "video file non-empty");

        std::ifstream stream(video, std::ios::binary);
        std::uint8_t  head[4]{};
        stream.read(reinterpret_cast<char *>(head), 4);
        check(head[0] == 0 && head[1] == 0 && head[2] == 0 && head[3] == 1, "video starts with Annex B start code");

        const std::vector<std::string> lines = read_lines(csv);
        check(lines.size() == FRAMES + 1, "csv has header + one line per frame");
        if (!lines.empty())
        {
            check(lines[0] == "t_ns,qw,qx,qy,qz", "csv header names");
        }
    }

    // save=false 丢弃文件。
    check(recorder.start(), "restart");
    recorder.push(make_frame(1), tools::time::now(), {0.0, 0.0, 0.0, 1.0});
    const std::optional<std::string> discarded = recorder.filename();
    recorder.stop(false);
    if (discarded.has_value())
    {
        check(!fs::exists(*discarded), "stop(false) discards video");
        check(!fs::exists(fs::path(*discarded).replace_extension(".csv")), "stop(false) discards csv");
    }

    // 目录不可用:start 返回 false 并给出原因。
    {
        const fs::path blocked = dir / "not_a_directory";
        std::ofstream(blocked.string()).put('\0');

        tools::record::RecordConfig bad;
        bad.directories = {(blocked / "sub").string()};

        tools::record::Recorder broken(bad);
        check(!broken.start(), "start fails on unusable directory");
        check(broken.status() != "not recording", "status explains failure");
    }

    fs::remove_all(dir);

    if (failures == 0)
    {
        std::cout << "record test passed\n";
        return 0;
    }
    std::cout << "record test failed: " << failures << "\n";
    return 1;
}
