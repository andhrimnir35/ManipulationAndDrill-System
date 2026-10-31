#pragma once

#include <stdint.h>

constexpr uint32_t as5600RetryBackoffUs(uint16_t consecutiveErrors) {
  return consecutiveErrors < 3
             ? 0UL
             : consecutiveErrors < 6
                   ? 10000UL
                   : consecutiveErrors < 12
                         ? 50000UL
                         : consecutiveErrors < 24 ? 250000UL : 1000000UL;
}

constexpr uint32_t nextI2cRecoveryBackoffMs(uint32_t currentBackoffMs) {
  constexpr uint32_t maximumBackoffMs = 10000UL;
  return currentBackoffMs >= maximumBackoffMs / 2UL
             ? maximumBackoffMs
             : currentBackoffMs * 2UL;
}

