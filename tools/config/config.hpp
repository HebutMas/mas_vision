#pragma once

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

namespace tools::config
{

// Config:一个 YAML 配置文件的只读视图。
class Config
{
public:
  // 加载失败(文件不存在 / YAML 语法错误)直接抛异常,让程序在启动阶段失败。
  explicit Config(const std::string & path)
  try : root_(YAML::LoadFile(path))
  {
  }
  catch (const YAML::Exception & error)
  {
    throw std::runtime_error("加载配置失败 " + path + ": " + error.what());
  }

  // 按键取值;键不存在或值为 null 时返回 fallback。
  template <typename T>
  [[nodiscard]] T get(const std::string & key, const T & fallback) const
  {
    const YAML::Node node = root_[key];
    if (!node || node.IsNull())
    {
      return fallback;
    }
    return node.as<T>();
  }

private:
  YAML::Node root_;
};

} // namespace tools::config
