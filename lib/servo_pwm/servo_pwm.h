#pragma once

#include <Arduino.h>

enum class ServoPulseSource : uint8_t {
  None = 0,
  FixedTest = 1,
  TubeTarget = 2,
  Wiggle = 3
};

struct ServoPwmDiagnostics {
  bool outputEnabled;
  bool configurationValid;
  uint32_t timerClockHz;
  uint32_t prescaler;
  uint32_t autoReload;
  uint32_t compare;
  uint32_t counter;
  uint32_t interruptEnable;
  uint32_t gpioMode;
  uint32_t gpioAlternateFunction;
  uint16_t commandedPulseUs;
  uint32_t writeCount;
  uint32_t lastChangeMs;
  ServoPulseSource lastSource;
};

constexpr uint16_t SERVO_PWM_MIN_US = 800;
constexpr uint16_t SERVO_PWM_MAX_US = 2200;
constexpr uint16_t SERVO_PWM_PERIOD_US = 20000;

bool servoPwmWriteMicroseconds(uint16_t pulseUs, ServoPulseSource source);
void servoPwmStop();
ServoPwmDiagnostics servoPwmDiagnostics();
const char *servoPulseSourceName(ServoPulseSource source);
