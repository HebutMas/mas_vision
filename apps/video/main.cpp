#include "apps/infantry/main_debug.hpp"
#include "hardware/serialport/serialport.hpp"
#include "modules/auto_armor/auto_armor.hpp"
#include "modules/auto_buff/auto_buff.hpp"
#include "tools/config/config.hpp"
#include "tools/debug/record/player.hpp"
#include "tools/exiter/exiter.hpp"
#include "tools/time/time.hpp"

#include <Eigen/Geometry>

#include <chrono>
#include <cstdint>
#include <exception>
#include <optional>
#include <string>
#include <thread>
#include <utility>

// 视频回放版步兵:图像源是 video.path 指定的视频(替代相机),串口照常收发。
// NOLINTNEXTLINE(bugprone-exception-escape): catch 内日志可能抛异常,进程直接退出即可。
int main(int argc, char **argv)
try
{
    (void)argc;
    (void)argv;
    // 配置文件
    const std::string           path = RM_CONFIG_PATH;
    const tools::config::Config config(path);

    // 远程调试
    infantry::Debug debug("rm_vision.video", config);
    tools::install_exit_handler();
    std::int64_t frame = 0;

    // 视频输入源:循环回放整个视频
    tools::record::PlayerConfig player_config;
    player_config.path = config.value<std::string>("video.path", "");
    if (player_config.path.empty())
    {
        tools::debug::log(tools::debug::Level::error, "video.path is empty");
        return 1;
    }
    tools::record::Player player(player_config);
    if (!player.is_open())
    {
        tools::debug::log(tools::debug::Level::error, "open video failed: " + player_config.path);
        return 1;
    }

    // 串口通信
    hardware::serialport::SerialPort serial(config);

    // 装甲板
    rm::armor::AutoAim auto_aim(config);
    // 能量机关
    rm::buff::AutoBuff auto_buff(config);

    // 串口状态:有数据就取最新的,没有则沿用上一次(初始为默认:红方自瞄 + 单位四元数)。
    hardware::serialport::ReceivePacket state{};
    state.mode = hardware::serialport::VisionMode::auto_aim_red;
    state.qw   = 1.0F;

    while (!tools::should_exit())
    {
        // 串口数据(非阻塞:没新数据就沿用上次状态)
        if (serial.is_open())
        {
            serial.frames().try_pop(state);
        }

        // 图像数据
        cv::Mat                image;
        tools::time::TimePoint timestamp;
        if (!player.read(image, timestamp))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 一帧的完整识别流程:按串口下发的 mode 在自瞄/能量机关之间切换,再送调试。
        const bool rune_mode    = hardware::serialport::is_rune_mode(state.mode);
        const bool red_mode     = hardware::serialport::is_red_mode(state.mode);
        const auto detect_begin = std::chrono::steady_clock::now();
        if (rune_mode)
        {
            const rm::buff::Color color = red_mode ? rm::buff::Color::red : rm::buff::Color::blue;

            auto         result    = auto_buff.process(image, color);
            const double detect_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - detect_begin).count();

            debug.push(image, frame++, timestamp, state, std::move(result.detection), detect_ms);
        }
        else
        {
            const std::optional<rm::armor::Color> enemy_color = red_mode ? rm::armor::Color::red : rm::armor::Color::blue;
            const Eigen::Quaterniond              q_imu       = state.quaternion().normalized().cast<double>();

            auto         result    = auto_aim.process(image, timestamp, q_imu, enemy_color);
            const double detect_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - detect_begin).count();

            debug.push(image, frame++, timestamp, state, std::move(result), detect_ms);
        }
    }

    return 0;
}
catch (const std::exception &error)
{
    tools::debug::log(tools::debug::Level::error, std::string("video exception: ") + error.what());
    return 1;
}
