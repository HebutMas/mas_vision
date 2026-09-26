#include "tools/latest_frame/latest_frame.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

namespace
{
int failures = 0;

void check(bool ok, const std::string & what)
{
  if (!ok)
  {
    std::cerr << "FAIL: " << what << "\n";
    ++failures;
  }
}
} // namespace

int main()
try
{
  // 覆盖语义:未取走时连续 push,只保留最后一个。
  {
    tools::LatestFrame<int> frame;
    frame.push(1);
    frame.push(2);
    frame.push(3);
    int value = 0;
    check(frame.try_pop(value) && value == 3, "push keeps only the latest value");
    check(!frame.try_pop(value), "slot is empty after pop");
  }

  // 阻塞等待:生产者稍后 push,消费者被唤醒。
  {
    tools::LatestFrame<int> frame;
    std::thread producer([&frame] { frame.push(42); });
    int value = 0;
    check(frame.wait(value) && value == 42, "wait returns the pushed value");
    producer.join();
  }

  // close 后 wait 立即返回 false,不再阻塞。
  {
    tools::LatestFrame<int> frame;
    frame.close();
    int value = 0;
    check(!frame.wait(value), "wait returns false after close");
  }

  // wait_for:无数据时超时返回 false;有数据时立即返回该值。
  {
    tools::LatestFrame<int> frame;
    int value = 0;
    check(!frame.wait_for(value, std::chrono::milliseconds(10)), "wait_for times out when empty");
    frame.push(7);
    check(frame.wait_for(value, std::chrono::milliseconds(10)) && value == 7,
          "wait_for returns the available value");
  }

  std::cout << (failures == 0 ? "latest_frame test passed\n" : "latest_frame test failed\n");
  return failures == 0 ? 0 : 1;
}
catch (const std::exception & error)
{
  std::cerr << "latest_frame_test error: " << error.what() << "\n";
  return 1;
}
