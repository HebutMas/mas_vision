#include "hardware/hikcamera/hikcamera.hpp"
#include "hardware/serialport/serialport.hpp"
#include "tools/config/config.hpp"
#include "tools/debug/debug.hpp"
#include "tools/exiter/exiter.hpp"
#include "tools/time/time.hpp"

#ifdef RM_DEBUG
#include "tools/debug/video/video_encoder.hpp"
#endif

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#ifdef RM_DEBUG
#include <memory>
#endif

int main(int argc, char **argv)
try
{
    (void)argc;
    (void)argv;
    // 配置文件
    const std::string           path = RM_CONFIG_PATH;
    const tools::config::Config config(path);

    // 串口通信
   // hardware::serialport::SerialPort serial(config);

    // hikcamera相机
    hardware::hikcamera::HikCamera camera(hardware::hikcamera::load_hikcamera_config(config));

    // 远程调试
    tools::debug::Sink debug("rm_vision.infantry");
    tools::install_exit_handler();

    std::int64_t frame = 0;
#ifdef RM_DEBUG
    std::unique_ptr<tools::video::VideoEncoder> encoder;
#endif

    while (!tools::should_exit())
    {
        // hardware::serialport::ReceivePacket state{};
        // if (serial.frames().try_pop(state))
        // {
        //     // debug输出
        //     if (debug.active())
        //     {
        //         const Eigen::Quaternionf q = state.quaternion();
        //         debug.set_frame(frame++);
        //         debug.set_time("time", tools::time::since_base(tools::time::now()));
        //         debug.data("serial/mode", state.mode);
        //         debug.data("serial/quaternion", {q.w(), q.x(), q.y(), q.z()});
        //     }
        // }

        // 相机:取最新一帧
        hardware::hikcamera::HikFrame hik_frame;
        if (camera.frames().try_pop(hik_frame) && !hik_frame.image.empty())
        {
#ifdef RM_DEBUG
            if (debug.active())
            {
                try
                {
                    if (!encoder)
                    {
                        tools::video::VideoEncoderConfig encoder_config;
                        encoder_config.width  = hik_frame.image.cols;
                        encoder_config.height = hik_frame.image.rows;
                        encoder = std::make_unique<tools::video::VideoEncoder>(encoder_config);
                    }
                    const auto timestamp = tools::time::since_base(hik_frame.timestamp);
                    debug.set_frame(frame++);
                    debug.set_time("time", timestamp);
                    for (auto &encoded : encoder->encode(hik_frame.image, timestamp))
                    {
                        debug.video("camera/image", std::move(encoded.data), encoded.keyframe);
                    }
                }
                catch (const std::exception &error)
                {
                    // 无 VAAPI 设备等异常:跳过视频输出,不算失败。
                    std::cout << "skip camera video output: " << error.what() << "\n";
                    encoder.reset();
                }
            }
#endif
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
