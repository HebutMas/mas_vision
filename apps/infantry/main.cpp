#include "apps/infantry/main_debug.hpp"
#include "hardware/hikcamera/hikcamera.hpp"
#include "hardware/serialport/serialport.hpp"
#include "modules/auto_armor/auto_armor.hpp"
#include "modules/auto_buff/auto_buff.hpp"
#include "tools/config/config.hpp"
#include "tools/exiter/exiter.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <optional>
#include <string>
#include <thread>
#include <utility>

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
    infantry::Debug debug("rm_vision.infantry");
    tools::install_exit_handler();
    std::int64_t frame = 0;

    // 串口通信
    hardware::serialport::SerialPort serial(config);

    // hikcamera 相机
    hardware::hikcamera::HikCamera hikcamera(hardware::hikcamera::load_hikcamera_config(config));

    // 装甲板
    rm::armor::AutoAim auto_aim(config);
    // 能量机关
    rm::buff::AutoBuff auto_buff(config);

    while (!tools::should_exit())
    {
        // 串口数据
        hardware::serialport::ReceivePacket state{};
        if (!serial.is_open() || !serial.frames().try_pop(state))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 图像数据
        hardware::hikcamera::HikFrame hik_frame;
        if (!hikcamera.frames().try_pop(hik_frame) || hik_frame.image.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 识别:按串口下发的 mode 在自瞄/能量机关之间切换
        const bool rune_mode    = hardware::serialport::is_rune_mode(state.mode);
        const bool red_mode     = hardware::serialport::is_red_mode(state.mode);
        const auto detect_begin = std::chrono::steady_clock::now();
        if (rune_mode)
        {
            const rm::buff::Color color = red_mode ? rm::buff::Color::red : rm::buff::Color::blue;

            auto         result    = auto_buff.process(hik_frame.image, color);
            const double detect_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - detect_begin).count();

            debug.push(hik_frame.image, frame++, hik_frame.timestamp, state, std::move(result.detection), detect_ms);
        }
        else
        {
            const std::optional<rm::armor::Color> enemy_color = red_mode ? rm::armor::Color::red : rm::armor::Color::blue;
            auto         result    = auto_aim.process(hik_frame.image, enemy_color);
            const double detect_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - detect_begin).count();

            debug.push(hik_frame.image, frame++, hik_frame.timestamp, state, std::move(result.detection), detect_ms);
        }
    }

    return 0;
}
catch (const std::exception &error)
{
    tools::debug::log(tools::debug::Level::error, std::string("infantry exception: ") + error.what());
    return 1;
}
