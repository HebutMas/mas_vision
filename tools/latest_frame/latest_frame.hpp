#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <utility>

namespace tools
{

// LatestFrame:单生产者 / 单消费者之间传递「最新一帧」的有界槽。
// 语义:push 永不阻塞,新值直接覆盖尚未取走的旧值(旧帧被丢弃);消费者 wait 拿到的
template <typename T>
class LatestFrame
{
public:
  // 覆盖式写入:直接替换槽内尚未取走的值,不阻塞。
  void push(T value)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      slot_ = std::move(value);
    }
    cv_.notify_one();
  }

  // 阻塞等待新值。成功时取出并清空槽,返回 true;已 close 返回 false。
  bool wait(T & out)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return slot_.has_value() || closed_; });
    if (closed_)
    {
      return false;
    }
    out = std::move(*slot_);
    slot_.reset();
    return true;
  }

  // 限时等待新值。成功返回 true;超时或已 close 返回 false。
  template <typename Rep, typename Period>
  bool wait_for(T & out, const std::chrono::duration<Rep, Period> & timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!cv_.wait_for(lock, timeout, [this] { return slot_.has_value() || closed_; }))
    {
      return false;
    }
    if (closed_)
    {
      return false;
    }
    out = std::move(*slot_);
    slot_.reset();
    return true;
  }

  // 非阻塞取最新值;槽为空返回 false。
  bool try_pop(T & out)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!slot_.has_value())
    {
      return false;
    }
    out = std::move(*slot_);
    slot_.reset();
    return true;
  }

  // 关闭:唤醒所有等待者,之后 wait 一律返回 false。
  void close()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    cv_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<T> slot_;
  bool closed_{false};
};

} // namespace tools
