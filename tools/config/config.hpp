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
    // 加载失败直接抛异常,让程序在启动阶段失败。
    explicit Config(const std::string &path)
    try : root_(YAML::LoadFile(path)) {}
    catch (const YAML::Exception &error)
    {
        throw std::runtime_error("load config failed " + path + ": " + error.what());
    }

    // 按 `.` 路径取必填项(如 "serial.port")
    template <typename T> [[nodiscard]] T require(const std::string &path) const
    {
        const YAML::Node node = find(path);
        if (!node || node.IsNull())
        {
            throw std::runtime_error("Missing configuration items" + path);
        }
        try
        {
            return node.as<T>();
        }
        catch (const YAML::Exception &error)
        {
            throw std::runtime_error("Configuration item type error " + path + ": " + error.what());
        }
    }

    // 按 `.` 路径取可选项:缺失或为空时返回默认值。
    template <typename T> [[nodiscard]] T value(const std::string &path, T fallback) const
    {
        const YAML::Node node = find(path);
        if (!node || node.IsNull())
        {
            return fallback;
        }
        try
        {
            return node.as<T>();
        }
        catch (const YAML::Exception &error)
        {
            throw std::runtime_error("Configuration item type error " + path + ": " + error.what());
        }
    }

  private:
    // 按 `.` 路径逐层下钻,返回命中的节点
    [[nodiscard]] YAML::Node find(const std::string &path) const
    {
        YAML::Node  node  = root_;
        std::size_t start = 0;
        while (true)
        {
            const std::size_t dot = path.find('.', start);
            // 用 reset 重新绑定到子节点
            node.reset(node[path.substr(start, dot - start)]);
            if (!node || node.IsNull())
            {
                return node;
            }
            if (dot == std::string::npos)
            {
                break;
            }
            start = dot + 1;
        }
        return node;
    }

    YAML::Node root_;
};

} // namespace tools::config
