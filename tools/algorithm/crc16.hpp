#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace tools::crc16
{
namespace detail
{

// 编译生成 256 项查表:CRC-16/CCITT-FALSE(poly 0x1021)。
constexpr std::array<std::uint16_t, 256> build_table() noexcept
{
    std::array<std::uint16_t, 256> table{};
    for (std::uint16_t i = 0; i < 256; ++i)
    {
        std::uint16_t crc = static_cast<std::uint16_t>(i << 8);
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 0x8000) ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021) : static_cast<std::uint16_t>(crc << 1);
        }
        table[i] = crc;
    }
    return table;
}

inline constexpr std::array<std::uint16_t, 256> TABLE = build_table();

} // namespace detail

// CRC-16/CCITT-FALSE:poly 0x1021,init 0xFFFF,输入/输出不反转,结果异或 0x0000;查表法逐字节处理
inline std::uint16_t checksum(const std::uint8_t *data, std::size_t size) noexcept
{
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < size; ++i)
    {
        crc = static_cast<std::uint16_t>((crc << 8) ^ detail::TABLE[(crc >> 8) ^ data[i]]);
    }
    return crc;
}

} // namespace tools::crc16
