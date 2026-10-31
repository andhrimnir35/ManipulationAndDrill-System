#include "reliability_math.h"

#include <assert.h>
#include <stdint.h>

int main() {
  assert(as5600RetryBackoffUs(0) == 0);
  assert(as5600RetryBackoffUs(2) == 0);
  assert(as5600RetryBackoffUs(3) == 10000);
  assert(as5600RetryBackoffUs(5) == 10000);
  assert(as5600RetryBackoffUs(6) == 50000);
  assert(as5600RetryBackoffUs(11) == 50000);
  assert(as5600RetryBackoffUs(12) == 250000);
  assert(as5600RetryBackoffUs(23) == 250000);
  assert(as5600RetryBackoffUs(24) == 1000000);
  assert(as5600RetryBackoffUs(UINT16_MAX) == 1000000);

  uint32_t recoveryBackoffMs = 500;
  const uint32_t expected[] = {1000, 2000, 4000, 8000, 10000, 10000};
  for (uint32_t value : expected) {
    recoveryBackoffMs = nextI2cRecoveryBackoffMs(recoveryBackoffMs);
    assert(recoveryBackoffMs == value);
  }

  // A permanently missing sensor may be probed quickly at first, but its
  // steady-state retry rate is bounded to one attempt per second.
  uint32_t nowUs = 0;
  uint32_t nextAllowedUs = 0;
  uint16_t errors = 0;
  uint16_t attempts = 0;
  for (; nowUs < 5000000UL; nowUs += 1000UL) {
    if (nextAllowedUs != 0 &&
        static_cast<int32_t>(nowUs - nextAllowedUs) < 0) {
      continue;
    }
    ++attempts;
    ++errors;
    nextAllowedUs = nowUs + as5600RetryBackoffUs(errors);
  }
  assert(attempts < 30);
  assert(errors >= 24);

  return 0;
}
