#include "hardware/serialport/serialport.hpp"
#include "tools/config/config.hpp"
#include "tools/debug/debug.hpp"
#include "tools/exiter/exiter.hpp"
#include "tools/time/time.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
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

    // 串口通信
    hardware::serialport::SerialPort serial(config);

    // 远程调试
    tools::debug::Sink debug("rm_vision.infantry");
    tools::install_exit_handler();

    std::int64_t frame = 0;

    while (!tools::should_exit())
    {
        hardware::serialport::ReceivePacket state{};
        if (serial.frames().try_pop(state))
        {
            // debug输出
            if (debug.active())
            {
                const Eigen::Quaternionf q = state.quaternion();
                debug.set_frame(frame++);
                debug.set_time("time", tools::time::since_base(tools::time::now()));
                debug.data("serial/mode", state.mode);
                debug.data("serial/quaternion", {q.w(), q.x(), q.y(), q.z()});
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    return 0;
}
catch (const std::exception &error)
{
    std::cerr << "infantry exception: " << error.what() << "\n";
    return 1;
}
