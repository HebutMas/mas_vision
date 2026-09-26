#include "hardware/hikcamera/hikcamera.hpp"
#if defined(RM_DEBUG)
#include "tools/debug/debug.hpp"
#endif

#include <chrono>
#include <exception>
#include <iostream>

#if defined(RM_DEBUG)
namespace
{
constexpr int SEND_SECONDS = 5;
} // namespace
#endif

// 测试:没有相机时应当抛异常并返回 0;接了相机则抓一帧验证输出格式。
// 调试构建下若相机出图,再把这一帧送到 Rerun Viewer 方便肉眼确认。
int main()
try
{
  const hardware::hikcamera::HikCameraConfig config;
  try
  {
    hardware::hikcamera::HikCamera camera(config);
    std::cout << "已打开海康 USB 相机,尝试抓取一帧\n";

    auto & frames = camera.frames();
    hardware::hikcamera::HikFrame frame;
    if (frames.wait_for(frame, std::chrono::milliseconds(2000)) && !frame.image.empty())
    {
      std::cout << "抓帧成功: " << frame.image.cols << "x" << frame.image.rows
                << " channels=" << frame.image.channels() << "\n";
#if defined(RM_DEBUG)
      tools::debug::Sink debug("rm_vision.hikcamera");
      if (debug.active())
      {
        // 连续发送 5 秒,每帧带上相对统一基准的时间戳,方便在 Viewer 里按真实时间回放。
        std::cout << "开始连续发送画面到 Rerun(5s)\n";
        const auto deadline = tools::time::now() + std::chrono::seconds(SEND_SECONDS);
        int index = 0;
        while (tools::time::now() < deadline && !frame.image.empty())
        {
          debug.set_frame(index);
          debug.set_time("time", tools::time::since_base(frame.timestamp));
          debug.image("camera/image", frame.image);
          ++index;
          if (!frames.wait_for(frame, std::chrono::milliseconds(2000)))
          {
            break;
          }
        }
        std::cout << "已发送 " << index << " 帧到 Rerun\n";
      }
      else
      {
        std::cout << "Rerun Viewer 不可用,跳过画面输出\n";
      }
#endif
    }
    else
    {
      std::cout << "超时未取到帧(相机可能未正常出图)\n";
    }
  }
  catch (const std::exception & error)
  {
    // 找不到相机属于正常情况:驱动会抛异常,这里视为通过。
    std::cout << "未打开相机: " << error.what() << "\n";
  }

  std::cout << "hikcamera test passed\n";
  return 0;
}
catch (const std::exception & error)
{
  std::cerr << "hikcamera_test error: " << error.what() << "\n";
  return 1;
}
