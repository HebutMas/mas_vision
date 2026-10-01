#include "apps/infantry/main_debug.hpp"
#include "hardware/hikcamera/hikcamera.hpp"
#include "hardware/serialport/serialport.hpp"
#include "modules/auto_armor/detection/detector.hpp"
#include "tools/config/config.hpp"
#include "tools/exiter/exiter.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <string>
#include <thread>

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

    // 装甲板检测
    rm::armor::Detector detector(rm::armor::load_detector_config(config));

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

        // 识别
        const rm::armor::Detector::Result detection = detector.detect(hik_frame.image);

        // TODO(track) TODO(fire)

        debug.push(hik_frame.image, frame++, hik_frame.timestamp, state, detection);
    }

    return 0;
}
catch (const std::exception &error)
{
    tools::debug::log(tools::debug::Level::error, std::string("infantry exception: ") + error.what());
    return 1;
}
