#include <Arduino.h>

#include "servo_pwm.h"

#ifdef SERVO_MINIMAL_TEST

HardwareSerial TestSerial(PA_3, PA_2);

namespace {

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr size_t COMMAND_CAPACITY = 64;
char commandBuffer[COMMAND_CAPACITY] = {};
size_t commandLength = 0;

void printDiagnostics() {
  const ServoPwmDiagnostics diag = servoPwmDiagnostics();
  TestSerial.print("SERVOTIMER PWM:");
  TestSerial.print(diag.outputEnabled ? 1 : 0);
  TestSerial.print(" VALID:");
  TestSerial.print(diag.configurationValid ? 1 : 0);
  TestSerial.print(" CLK:");
  TestSerial.print(diag.timerClockHz);
  TestSerial.print(" PSC:");
  TestSerial.print(diag.prescaler);
  TestSerial.print(" ARR:");
  TestSerial.print(diag.autoReload);
  TestSerial.print(" CCR1:");
  TestSerial.print(diag.compare);
  TestSerial.print(" DIER:");
  TestSerial.print(diag.interruptEnable);
  TestSerial.print(" PA0MODE:");
  TestSerial.print(diag.gpioMode);
  TestSerial.print(" PA0AF:");
  TestSerial.print(diag.gpioAlternateFunction);
  TestSerial.print(" PULSE:");
  TestSerial.print(diag.commandedPulseUs);
  TestSerial.print(" WRITES:");
  TestSerial.print(diag.writeCount);
  TestSerial.print(" LASTCHANGE_MS:");
  TestSerial.print(diag.lastChangeMs);
  TestSerial.print(" SOURCE:");
  TestSerial.println(servoPulseSourceName(diag.lastSource));
}

void processCommand() {
  commandBuffer[commandLength] = '\0';
  commandLength = 0;

  char *save = nullptr;
  char *first = strtok_r(commandBuffer, " \t", &save);
  if (first == nullptr) {
    return;
  }
  for (char *p = first; *p != '\0'; ++p) {
    *p = static_cast<char>(tolower(static_cast<unsigned char>(*p)));
  }

  if (strcmp(first, "servo") == 0) {
    char *second = strtok_r(nullptr, " \t", &save);
    char *third = strtok_r(nullptr, " \t", &save);
    if (second != nullptr && strcmp(second, "diag") == 0) {
      printDiagnostics();
      return;
    }
    if (second == nullptr || strcmp(second, "pulse") != 0 ||
        third == nullptr) {
      TestSerial.println("ERR USAGE_SERVO_PULSE_1000_1500_2000_OFF");
      return;
    }
    if (strcmp(third, "off") == 0) {
      servoPwmStop();
      TestSerial.println("OK SERVO_PWM_OFF");
      printDiagnostics();
      return;
    }
    char *end = nullptr;
    const long pulseUs = strtol(third, &end, 10);
    if (end == third || *end != '\0' ||
        (pulseUs != 1000 && pulseUs != 1500 && pulseUs != 2000)) {
      TestSerial.println("ERR FIXED_TEST_PULSE_MUST_BE_1000_1500_2000");
      return;
    }
    if (!servoPwmWriteMicroseconds(
            static_cast<uint16_t>(pulseUs),
            ServoPulseSource::FixedTest)) {
      TestSerial.println("ERR SERVO_TIMER_CONFIGURATION");
      return;
    }
    TestSerial.print("OK SERVO_FIXED_PULSE_US=");
    TestSerial.println(pulseUs);
    printDiagnostics();
    return;
  }

  if (strcmp(first, "stopall") == 0 || strcmp(first, "stop") == 0) {
    servoPwmStop();
    TestSerial.println("OK STOPALL SERVO_PWM_OFF");
    return;
  }
  if (strcmp(first, "hb") == 0) {
    return;
  }
  if (strcmp(first, "mode") == 0) {
    TestSerial.println("OK MODE DRILL MINIMAL_SERVO_TEST");
    return;
  }
  if (strcmp(first, "help") == 0) {
    TestSerial.println(
        "servo pulse 1000|1500|2000|off; servo diag; stopall");
    return;
  }
  TestSerial.println("ERR MINIMAL_SERVO_TEST_COMMAND");
}

void serviceSerial() {
  while (TestSerial.available() > 0) {
    const char incoming = static_cast<char>(TestSerial.read());
    if (incoming == '\r') {
      continue;
    }
    if (incoming == '\n') {
      processCommand();
      continue;
    }
    if (commandLength < COMMAND_CAPACITY - 1U) {
      commandBuffer[commandLength++] = incoming;
    } else {
      commandLength = 0;
      TestSerial.println("ERR COMMAND_TOO_LONG");
    }
  }
}

}  // namespace

void setup() {
  servoPwmStop();
  TestSerial.begin(SERIAL_BAUD);
  delay(50);
  TestSerial.println("OK BOOT MINIMAL_SERVO_TEST PWM_OFF");
  TestSerial.println(
      "OK COMMANDS servo pulse 1000|1500|2000|off; servo diag; stopall");
}

void loop() {
  serviceSerial();
}

#endif
