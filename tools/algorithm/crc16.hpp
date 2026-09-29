#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace tools::crc16
{
namespace detail
{

// 编译生成 256 项查表:poly 0x8408
constexpr std::array<std::uint16_t, 256> build_table() noexcept
{
    std::array<std::uint16_t, 256> table{};
    for (std::uint16_t i = 0; i < 256; ++i)
    {
        std::uint16_t crc = i;
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 1) ? static_cast<std::uint16_t>((crc >> 1) ^ 0x8408) : static_cast<std::uint16_t>(crc >> 1);
        }
        table[i] = crc;
    }
    return table;
}

inline constexpr std::array<std::uint16_t, 256> TABLE = build_table();

} // namespace detail

// CRC-16/MCRF4XX :poly 0x8408,init 0xFFFF,输入/输出反转,xorout 0x0000;
inline std::uint16_t checksum(const std::uint8_t *data, std::size_t size) noexcept
{
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < size; ++i)
    {
        crc = static_cast<std::uint16_t>((crc >> 8) ^ detail::TABLE[(crc ^ data[i]) & 0xFF]);
    }
    return crc;
}

} // namespace tools::crc16
