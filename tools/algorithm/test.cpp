#include "tools/algorithm/crc16.hpp"
#include "tools/algorithm/quaternion_buffer.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>

namespace
{
int failures = 0;

void check(bool ok, const char *what)
{
    if (!ok)
    {
        std::cerr << "FAIL: " << what << "\n";
        ++failures;
    }
}

bool near(float a, float b, float eps = 1e-4F) { return std::fabs(a - b) <= eps; }
} // namespace

int main()
{
    // CRC-16/CCITT-FALSE 标准测试向量。
    const auto *data = reinterpret_cast<const std::uint8_t *>("123456789");
    check(tools::crc16::checksum(data, 9) == 0x29B1, "crc16 known vector");
    check(tools::crc16::checksum(nullptr, 0) == 0xFFFF, "crc16 empty init");

    // 四元数缓冲:空 -> 单位;0 → 90°(绕 Z)的中点应为 45°;越界取端点。
    tools::TimedQuaternionBuffer<8> buffer;
    check(near(buffer.at(tools::time::now()).w(), 1.0F), "empty -> identity");

    const auto  t0 = tools::time::base();
    const float h  = std::sqrt(0.5F);
    buffer.push(t0, Eigen::Quaternionf::Identity());
    buffer.push(t0 + std::chrono::milliseconds(10), Eigen::Quaternionf(h, 0.0F, 0.0F, h)); // 90° about Z

    const float eighth = 0.3926991F; // pi/8 = 22.5°
    const auto  mid    = buffer.at(t0 + std::chrono::milliseconds(5));
    check(near(mid.w(), std::cos(eighth)) && near(mid.z(), std::sin(eighth)), "slerp midpoint ~45deg");

    const auto before = buffer.at(t0 - std::chrono::seconds(1));
    check(near(before.w(), 1.0F) && near(before.z(), 0.0F), "clamp oldest");
    const auto after = buffer.at(t0 + std::chrono::seconds(1));
    check(near(after.w(), h) && near(after.z(), h), "clamp newest");

    // clear 后回到单位四元数
    buffer.clear();
    check(near(buffer.at(t0).w(), 1.0F), "clear -> identity");

    if (failures != 0)
    {
        std::cerr << "algorithm test failed: " << failures << "\n";
        return 1;
    }
    std::cout << "algorithm test passed\n";
    return 0;
}
