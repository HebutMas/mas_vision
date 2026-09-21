// 远程调试链路的测试
// 无论开发机上有没有 Viewer,这个测试都必须通过:
// Viewer 在线   :发送若干帧合成数据,验证 编码 / 发送 路径不崩、能正常收尾。
// Viewer 不在线 :验证 Sink 降级 —— 构造不阻塞,之后所有调用都是安全空操作。
// 后者是回归重点:Rerun 的 gRPC sink 在连不上时会等待约 5 秒,进程退出时甚至可能卡死。
// Sink 的 TCP 预探测必须把构造耗时压到几百毫秒以内。
//
// 手动跑(指定开发机地址可顺便验证在线路径):
//   ./build/tools/debug_test rerun+http://<开发机IP>:9876/proxy
#include "tools/debug/debug.hpp"

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace
{
constexpr double PI = 3.14159265358979323846;

// 离线时允许的最大构造耗时。预探测超时是 300ms,这里放宽到 2s;
// 一旦预探测失效(退化成 gRPC 的 ~5s 阻塞),断言就会失败。
constexpr double MAX_OFFLINE_SINK_SECONDS = 2.0;

// 在线时发送的帧数;30 帧 x 30ms ≈ 1s,够跑通链路又不拖慢 ctest。
constexpr int ONLINE_FRAMES = 30;

// 生成一帧合成图像:一个圆周轨迹上的圆点,外加一个绕它旋转的方框。
// 把方框顶点作为装甲板四点返回,便于在 Viewer 里检查几何叠加。
cv::Mat draw_frame(int frame, std::vector<cv::Point> & armor_corners, cv::Point2f & marker)
{
  const double t = frame * 0.1;
  const cv::Point2f center{320.0F, 240.0F};
  marker = {center.x + static_cast<float>(150.0 * std::cos(t)),
            center.y + static_cast<float>(150.0 * std::sin(t))};

  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(20, 20, 20));
  cv::circle(image, center, 150, {0, 120, 0}, 1);
  cv::circle(image, marker, 8, {0, 0, 255}, cv::FILLED);

  armor_corners.clear();
  armor_corners.reserve(4);
  for (int i = 0; i < 4; ++i)
  {
    const double theta = t + (static_cast<double>(i) * PI / 2.0) + (PI / 4.0);
    const int x = static_cast<int>(marker.x + (45.0F * std::cos(theta)));
    const int y = static_cast<int>(marker.y + (45.0F * std::sin(theta)));
    armor_corners.emplace_back(x, y);
  }
  cv::polylines(image, armor_corners, true, {0, 255, 0}, 2);
  return image;
}

// Viewer 不在线时的用例:构造必须快,且之后所有调用都得是安全空操作。
int run_offline(const std::string & address, double construct_seconds)
{
  if (construct_seconds > MAX_OFFLINE_SINK_SECONDS)
  {
    std::cerr << "Sink structure timeout " << construct_seconds << "s,exceeding the limit " << MAX_OFFLINE_SINK_SECONDS
              << "s(pre-detection failure)\n";
    return 1;
  }

  tools::debug::Sink sink("rm_vision.debug_test", address);
  sink.set_frame(0);
  sink.image("camera/image", cv::Mat::zeros(8, 8, CV_8UC3));
  sink.data("test/value", 1.0);
  sink.data("test/values", {1.0, 2.0}, {"a", "b"});
  sink.log(tools::debug::Level::info, "offline");

  std::cout << "Viewer offline,Sink downgraded(structure time " << construct_seconds << "s)\n";
  return 0;
}

// Viewer 在线时的用例:发送合成数据,确认编码与发送路径通。
int run_online(tools::debug::Sink & sink)
{
  for (int frame = 0; frame < ONLINE_FRAMES; ++frame)
  {
    sink.set_frame(frame);

    std::vector<cv::Point> armor_corners;
    cv::Point2f marker;
    const cv::Mat image = draw_frame(frame, armor_corners, marker);

    // 图像:内部 JPEG 压缩后发送,调用方只负责画。
    sink.image("camera/image", image);
    // 数据:x / y 两个分量,在 Viewer 里展开成 test > marker > x / y 两条曲线。
    sink.data("test/marker", {static_cast<double>(marker.x), static_cast<double>(marker.y)},
              {"x", "y"});
    // 日志:带等级。
    sink.log(tools::debug::Level::info, "debug test frame " + std::to_string(frame));

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  std::cout << "already sent " << ONLINE_FRAMES << " frames\n";
  return 0;
}
} // namespace

int main(int argc, char ** argv)
try
{
  // 第 1 个参数可选,用来指定 Viewer 地址;默认发往本机。
  const std::string address = argc > 1 ? argv[1] : tools::debug::DEFAULT_ADDRESS;

  const auto begin = std::chrono::steady_clock::now();
  tools::debug::Sink sink("rm_vision.debug_test", address);
  const double construct_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();

  // 离线时 Sink 已经 inactive,必须重新构造一个来测空操作路径,以免干扰计时。
  return sink.active() ? run_online(sink) : run_offline(address, construct_seconds);
}
catch (const std::exception & error)
{
  std::cerr << "debug_test error: " << error.what() << "\n";
  return 1;
}
