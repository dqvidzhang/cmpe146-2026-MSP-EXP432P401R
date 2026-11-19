#pragma once

#include <array>
#include <cstdint>

namespace msp432_uart_detail {
struct baud_dividers
{
  bool valid = false;
  std::uint16_t prescaler = 0;
  std::uint16_t modulation = 0;
};

// TI eUSCI_A UART baud calculation, including the fractional-N UCBRS table.
// Integer arithmetic avoids floating-point code in the driver/constructor.
constexpr baud_dividers calculate_baud(std::uint32_t p_clock,
                                       std::uint32_t p_baud)
{
  if (p_baud == 0 || p_clock < p_baud) {
    return {};
  }
  std::uint32_t const ratio = p_clock / p_baud;
  bool const oversampling = ratio >= 16;
  std::uint32_t const prescaler = oversampling ? ratio / 16 : ratio;
  if (prescaler == 0 || prescaler > 0xFFFF) {
    return {};
  }
  struct modulation_entry
  {
    std::uint16_t fraction;
    std::uint8_t pattern;
  };
  constexpr std::array<modulation_entry, 36> table = { {
    { 0, 0x00 },    { 529, 0x01 },  { 715, 0x02 },  { 835, 0x04 },
    { 1001, 0x08 }, { 1252, 0x10 }, { 1430, 0x20 }, { 1670, 0x11 },
    { 2147, 0x21 }, { 2224, 0x22 }, { 2503, 0x44 }, { 3000, 0x25 },
    { 3335, 0x49 }, { 3575, 0x4A }, { 3753, 0x52 }, { 4003, 0x92 },
    { 4286, 0x53 }, { 4378, 0x55 }, { 5002, 0xAA }, { 5715, 0x6B },
    { 6003, 0xAD }, { 6254, 0xB5 }, { 6432, 0xB6 }, { 6667, 0xD6 },
    { 7001, 0xB7 }, { 7147, 0xBB }, { 7503, 0xDD }, { 7861, 0xED },
    { 8004, 0xEE }, { 8333, 0xBF }, { 8464, 0xDF }, { 8572, 0xEF },
    { 8751, 0xF7 }, { 9004, 0xFB }, { 9170, 0xFD }, { 9288, 0xFE },
  } };
  std::uint64_t const fraction =
    static_cast<std::uint64_t>(p_clock % p_baud) * 10000;
  std::uint8_t second_modulation = 0;
  for (auto const entry : table) {
    if (fraction < static_cast<std::uint64_t>(entry.fraction) * p_baud) {
      break;
    }
    second_modulation = entry.pattern;
  }
  auto const first_modulation = oversampling ? ratio % 16 : 0;
  auto const modulation = static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(second_modulation) << 8) |
    (first_modulation << 4) | (oversampling ? 0x01 : 0x00));
  return { true, static_cast<std::uint16_t>(prescaler), modulation };
}
}  // namespace msp432_uart_detail
