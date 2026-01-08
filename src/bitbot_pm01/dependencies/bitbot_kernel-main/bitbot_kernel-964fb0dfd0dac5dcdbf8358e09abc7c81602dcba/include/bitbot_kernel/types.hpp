#pragma once

#include <stdint.h>
#include <variant>

namespace bitbot
{
  using Number = std::variant<uint8_t, uint16_t, uint32_t, int64_t, uint64_t, double>;

  using EventId = uint32_t;
  using EventValue = int64_t;
  using StateId = uint32_t;

  enum class KeyboardEvent : EventValue
  {
    Down = 1,
    Up = 2
  };
  
}