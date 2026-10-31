#include <Arduino.h>
#include <EEPROM.h>
#include <IWatchdog.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "breakpoint_math.h"
#include "reliability_math.h"
#include "servo_pwm.h"

#ifndef BREAKPOINT_DEBUG
#define BREAKPOINT_DEBUG 0
#endif

#ifndef I2C_TIMEOUT_TICK
#error "I2C_TIMEOUT_TICK must be defined so Wire transactions stay bounded"
#endif
static_assert(I2C_TIMEOUT_TICK >= 1 && I2C_TIMEOUT_TICK <= 20,
              "I2C timeout must remain short enough for safety services");

// STM32F401 BlackPill firmware for the rover manipulation and drill system.
// H7 <-> F401 command link: USART2, PA3=RX, PA2=TX.

HardwareSerial LinkSerial(PA_3, PA_2);

constexpr uint32_t LINK_BAUD = 115200;
constexpr uint8_t AXIS_COUNT = 6;
constexpr uint8_t MOTOR_COUNT = 6;
constexpr uint8_t DEFAULT_PWM = 100;
constexpr uint8_t MAX_PWM = 255;
constexpr uint8_t SENDPWM_PERCENT_MAX = 100;
constexpr uint16_t DEFAULT_GAIN = 1000;
constexpr uint16_t DEFAULT_DEAD = 50;
constexpr uint16_t DEFAULT_KP = 3000;
constexpr uint16_t DEFAULT_KD = 0;
constexpr uint8_t DEFAULT_MIN_PWM = 35;
constexpr uint32_t SETTINGS_MAGIC = 0x4D445331UL; // "MDS1"
constexpr uint16_t SETTINGS_VERSION = 13;
constexpr uint16_t BRAKE_PULSE_MS = 120;
constexpr float POSITION_TOLERANCE_DEG = 0.8f;
constexpr uint16_t DEFAULT_STREAM_MS = 250;
constexpr uint16_t DEFAULT_STATUS_MS = 250;
constexpr uint16_t MIN_STATUS_MS = 100;
constexpr uint16_t MAX_STATUS_MS = 60000;

constexpr uint32_t HARDWARE_WATCHDOG_TIMEOUT_US = 4000000UL;
constexpr uint16_t HARDWARE_WATCHDOG_RELOAD_INTERVAL_MS = 250;
constexpr uint16_t SERIAL_SERVICE_BYTE_BUDGET = 192;
constexpr int16_t KEYBIND_DISABLED = -1;
constexpr int16_t DEFAULT_HAT0_UP_ID = 1000;
constexpr int16_t DEFAULT_HAT0_DOWN_ID = 1001;
constexpr int16_t DEFAULT_HAT0_LEFT_ID = 1002;
constexpr int16_t DEFAULT_HAT0_RIGHT_ID = 1003;
constexpr uint8_t SERVO_PIN = PA0;
constexpr uint8_t SERVO_TIMER_CHANNEL = 1;
constexpr uint16_t SERVO_PERIOD_US = 20000;
constexpr uint16_t SERVO_CENTER_US = 1500;
constexpr uint16_t SERVO_MIN_US = 800;
constexpr uint16_t SERVO_MAX_US = 2200;
static_assert(SERVO_MIN_US == SERVO_PWM_MIN_US,
              "servo PWM driver minimum must match firmware");
static_assert(SERVO_MAX_US == SERVO_PWM_MAX_US,
              "servo PWM driver maximum must match firmware");
static_assert(SERVO_PERIOD_US == SERVO_PWM_PERIOD_US,
              "servo PWM driver period must match firmware");
// Power HD 1235MG specifies 90 degrees over the standard 1000..2000 us
// interval and approximately 165 degrees over the extended 800..2200 us
// interval. The outer 200 us regions therefore need a different scale.
constexpr uint16_t SERVO_STANDARD_MIN_US = 1000;
constexpr uint16_t SERVO_STANDARD_MAX_US = 2000;
constexpr float SERVO_STANDARD_HALF_TRAVEL_DEG = 45.0f;
constexpr float SERVO_MAX_ABSOLUTE_ANGLE_DEG = 82.5f;
constexpr uint8_t SERVO_STATE_MIN = 1;
constexpr uint8_t SERVO_STATE_MAX = 5;
constexpr uint8_t SERVO_STATE_COUNT =
    SERVO_STATE_MAX - SERVO_STATE_MIN + 1;
constexpr uint8_t SERVO_STATE_VALID_MASK =
    static_cast<uint8_t>((1U << SERVO_STATE_COUNT) - 1U);
constexpr uint8_t DEFAULT_SERVO_STATE_ANGLE_DEG = 45;
constexpr uint8_t DEFAULT_SERVO_WIGGLE_ANGLE_DEG = 5;
constexpr uint8_t MAX_SERVO_STATE_ANGLE_DEG = 55;
constexpr uint8_t MAX_SERVO_WIGGLE_ANGLE_DEG = 30;
constexpr uint8_t DEFAULT_SERVO_SPEED_DPS = 30;
constexpr uint8_t MIN_SERVO_SPEED_DPS = 5;
constexpr uint8_t MAX_SERVO_SPEED_DPS = 120;
// State transitions, production fixed-pulse moves and the optional wiggle
// feature advance at most once per 50 Hz servo frame.
constexpr uint16_t SERVO_RAMP_INTERVAL_MS = 20;
constexpr uint16_t DEFAULT_SERVO_MANUAL_STEP_US = 10;
constexpr uint16_t MAX_SERVO_MANUAL_STEP_US = 100;
constexpr uint16_t SERVO_WIGGLE_TOTAL_MS = 2000;
constexpr uint16_t SERVO_WIGGLE_STEP_MS = 200;
constexpr uint8_t DRILL_LED_COUNT = 4;
constexpr uint8_t DRILL_LED_PINS[DRILL_LED_COUNT] = {PB4, PB13, PB14, PB15};
constexpr float BRAKE_MIN_LEAD_DEG = 1.0f;
constexpr float BRAKE_MAX_LEAD_DEG = 45.0f;
constexpr uint16_t DEFAULT_BP_REACTION_MS = 40;
constexpr uint16_t DEFAULT_BP_DECEL_DPS2[3] = {500, 350, 650};
constexpr float DEFAULT_BP_LEAD_MARGIN_DEG[3] = {0.6f, 0.8f, 0.6f};
constexpr float DEFAULT_BP_RECOVERY_CLEARANCE_DEG[3] = {0.8f, 1.0f, 0.8f};
constexpr uint8_t DEFAULT_BP_RECOVERY_PWM[3] = {70, 90, 65};
constexpr uint16_t DEFAULT_BP_RECOVERY_TIMEOUT_MS[3] = {2500, 3500, 2500};
constexpr uint16_t BP_SETTLE_MIN_MS = 40;
constexpr uint16_t BP_SETTLE_REBRAKE_MS = 750;
constexpr uint8_t BP_SETTLE_SAMPLE_COUNT = 5;
constexpr float BP_SETTLE_VELOCITY_DPS = 4.0f;
constexpr float BP_APPROACH_MIN_VELOCITY_DPS = 1.0f;
constexpr float BP_MIN_RECOVERY_TOLERANCE_DEG = 0.2f;
constexpr float BP_MIN_CLEARANCE_DEG = 0.5f;
constexpr uint16_t BP_RECOVERY_NO_PROGRESS_MS = 750;
constexpr float BP_RECOVERY_PROGRESS_DEG = 0.05f;
constexpr uint8_t BP_SLOT_COUNT = 4;
constexpr uint8_t BP_BOUNDARY_COUNT = BP_SLOT_COUNT;
constexpr uint8_t BP_INVALID_BOUNDARY = 255;
constexpr uint8_t TCA9548A_ADDR = 0x70;
constexpr uint8_t TCA9548A_SDA_PIN = PB9;
constexpr uint8_t TCA9548A_SCL_PIN = PB8;
constexpr uint8_t AS5600_ADDR = 0x36;
constexpr uint8_t AS5600_STATUS_REG = 0x0B;
constexpr uint8_t AS5600_RAW_ANGLE_REG = 0x0C;
constexpr uint8_t AS5600_CHANNEL_DISABLED = 255;
constexpr uint32_t AS5600_I2C_CLOCK_HZ = 400000;
constexpr float AS5600_DEGREES_PER_COUNT = 360.0f / 4096.0f;
constexpr float AS5600_VELOCITY_CUTOFF_HZ = 12.0f;
constexpr float AS5600_MAX_VALID_VELOCITY_DPS = 12000.0f;
constexpr uint32_t AS5600_MAX_VELOCITY_DT_US = 50000;
constexpr uint32_t AS5600_STALE_TIMEOUT_US = 50000;
constexpr uint8_t I2C_RECOVERY_ERROR_THRESHOLD = 8;
constexpr uint32_t I2C_RECOVERY_INITIAL_BACKOFF_MS = 500;
// One mux channel is serviced per 1 ms slot: three enabled axes each update at ~333 Hz.
constexpr uint32_t AS5600_UPDATE_SLOT_PERIOD_US = 1000;
constexpr uint32_t MOTION_CONTROL_PERIOD_US = 5000;
constexpr uint16_t POSITION_OUTPUT_RAMP_PWM_PER_SEC = 700;
constexpr uint16_t POSITION_REVERSE_COAST_MS = 35;
constexpr float HOLD_REENGAGE_MULTIPLIER = 2.5f;
constexpr float HOLD_REENGAGE_MIN_DEG = 1.0f;
constexpr float HOLD_SETTLE_VELOCITY_DPS = 4.0f;

enum class SystemMode : uint8_t { Safe = 0, Arm = 1, Drill = 2, ArmTest = 3 };
enum class Direction : int8_t { Stop = 0, Forward = 1, Backward = -1 };
enum class StopMode : uint8_t { Coast = 0, Brake = 1, Hold = 2, Hybrid = 3 };
enum class BrakePointPhase : uint8_t {
  Inactive = 0,
  Braking = 1,
  Settling = 2,
  Recovering = 3,
  Guarded = 4,
  Fault = 5
};
enum class BrakePointOutputAction : uint8_t { Allow = 0, Ignore = 1, Brake = 2 };
enum class BrakePointFaultReason : uint8_t {
  None = 0,
  RecoveryTimeout = 1,
  RecoveryNoProgress = 2,
  RecoveryBlocked = 3,
  RuntimeConflict = 4
};
enum class ConfirmAction : uint8_t {
  None = 0,
  ModeArm = 1,
  ModeDrill = 2,
  ModeArmTest = 3,
  FactoryReset = 4
};

struct MotorPins {
  uint8_t rpwm;
  uint8_t lpwm;
  uint8_t en;
};

struct MotorRuntime {
  Direction direction = Direction::Stop;
  uint8_t pwm = 0;
  bool braking = false;
  uint32_t brakeUntilMs = 0;
};

struct AxisConfig {
  uint8_t invert;
  uint8_t maxPwm;
  uint8_t defaultPwm;
  uint16_t posGain;
  uint16_t negGain;
  uint16_t dead;
  uint8_t stopMode;
  uint16_t kp;
  uint16_t kd;
  uint8_t minPwm;
  float toleranceDeg;
  float zeroOffsetDeg;
  int16_t keybindForward;
  int16_t keybindBackward;
  uint8_t as5600Channel;
};

struct DrillConfig {
  uint8_t invert;
  uint8_t maxPwm;
  uint8_t defaultPwm;
};

struct ElevatorConfig {
  uint8_t invert;
  uint8_t maxPwm;
  uint8_t defaultPwm;
  uint8_t sendPwmPercent[2]; // BTS4, BTS5; 100 means 1.00.
};

struct LegacyServoConfigV11 {
  uint8_t stateAngleDeg;
  uint8_t wiggleAngleDeg;
  uint8_t invert;
  int16_t keybindNext;
  int16_t keybindPrevious;
  uint8_t speedDegPerSec;
};

// Exact version-12 servo layout, retained to add the fifth state without
// losing the four existing calibrations.
struct LegacyServoConfigV12 {
  uint8_t stateAngleDeg;
  uint8_t wiggleAngleDeg;
  uint8_t invert;
  int16_t keybindNext;
  int16_t keybindPrevious;
  uint8_t speedDegPerSec;
  uint16_t statePulseUs[4];
  uint8_t statePulseValidMask;
};

struct ServoConfig {
  uint8_t stateAngleDeg;
  uint8_t wiggleAngleDeg;
  uint8_t invert;
  int16_t keybindNext;
  int16_t keybindPrevious;
  uint8_t speedDegPerSec;
  uint16_t statePulseUs[SERVO_STATE_COUNT];
  uint8_t statePulseValidMask;
};

struct BrakePointTuning {
  uint16_t reactionMs;
  uint16_t decelDps2;
  float leadMarginDeg;
  float recoveryClearanceDeg;
  uint8_t recoveryMaxPwm;
  uint16_t recoveryTimeoutMs;
};

struct BrakePointSlot {
  float angleDeg;
  uint8_t mode;
};

struct LegacyRemovedRangeV7 {
  float startDeg;
  float endDeg;
  uint8_t enabled;
};

// Exact version-5/6 axis layout, retained only for EEPROM migration.
struct LegacyAxisConfigV6 {
  uint8_t invert;
  uint8_t maxPwm;
  uint8_t defaultPwm;
  uint16_t posGain;
  uint16_t negGain;
  uint16_t dead;
  uint8_t stopMode;
  uint16_t kp;
  uint16_t kd;
  uint8_t minPwm;
  float toleranceDeg;
  float zeroOffsetDeg;
  float minAngleDeg;
  float maxAngleDeg;
  uint8_t limitsEnabled;
  float brakePointDeg;
  uint8_t brakePointEnabled;
  int16_t keybindForward;
  int16_t keybindBackward;
  uint8_t as5600Channel;
};

struct PersistentSettingsV5 {
  uint32_t magic;
  uint16_t version;
  LegacyAxisConfigV6 axis[AXIS_COUNT];
  DrillConfig elevator;
  DrillConfig drill;
  uint32_t legacyUnusedWord;
  uint32_t checksum;
};

// Exact version-6 layout, retained only to migrate the previous BP model.
struct PersistentSettingsV6 {
  uint32_t magic;
  uint16_t version;
  LegacyAxisConfigV6 axis[AXIS_COUNT];
  DrillConfig elevator;
  DrillConfig drill;
  uint32_t legacyUnusedWord;
  BrakePointTuning brakePoint[3];
  uint32_t checksum;
};

// Exact version-7 layout, retained only to migrate four-slot BP settings.
struct PersistentSettingsV7 {
  uint32_t magic;
  uint16_t version;
  AxisConfig axis[AXIS_COUNT];
  DrillConfig elevator;
  DrillConfig drill;
  uint32_t legacyUnusedWord;
  BrakePointTuning brakePoint[3];
  BrakePointSlot brakePointSlots[3][BP_SLOT_COUNT];
  LegacyRemovedRangeV7 removedRange[3];
  uint32_t checksum;
};

// Exact version-8 layout, retained to add elevator sendpwm without losing settings.
struct PersistentSettingsV8 {
  uint32_t magic;
  uint16_t version;
  AxisConfig axis[AXIS_COUNT];
  DrillConfig elevator;
  DrillConfig drill;
  BrakePointTuning brakePoint[3];
  BrakePointSlot brakePointSlots[3][BP_SLOT_COUNT];
  uint32_t checksum;
};

// Exact version-9 layout, retained to add drill servo settings safely.
struct PersistentSettingsV9 {
  uint32_t magic;
  uint16_t version;
  AxisConfig axis[AXIS_COUNT];
  ElevatorConfig elevator;
  DrillConfig drill;
  BrakePointTuning brakePoint[3];
  BrakePointSlot brakePointSlots[3][BP_SLOT_COUNT];
  uint32_t checksum;
};

// Exact version-10 layout, retained to add ramped servo speed safely.
struct LegacyServoConfigV10 {
  uint8_t stateAngleDeg;
  uint8_t wiggleAngleDeg;
  uint8_t invert;
  int16_t keybindNext;
  int16_t keybindPrevious;
};

struct PersistentSettingsV10 {
  uint32_t magic;
  uint16_t version;
  AxisConfig axis[AXIS_COUNT];
  ElevatorConfig elevator;
  DrillConfig drill;
  LegacyServoConfigV10 servo;
  BrakePointTuning brakePoint[3];
  BrakePointSlot brakePointSlots[3][BP_SLOT_COUNT];
  uint32_t checksum;
};

// Exact version-11 layout, retained to add per-tube servo calibration.
struct PersistentSettingsV11 {
  uint32_t magic;
  uint16_t version;
  AxisConfig axis[AXIS_COUNT];
  ElevatorConfig elevator;
  DrillConfig drill;
  LegacyServoConfigV11 servo;
  BrakePointTuning brakePoint[3];
  BrakePointSlot brakePointSlots[3][BP_SLOT_COUNT];
  uint32_t checksum;
};

struct PersistentSettingsV12 {
  uint32_t magic;
  uint16_t version;
  AxisConfig axis[AXIS_COUNT];
  ElevatorConfig elevator;
  DrillConfig drill;
  LegacyServoConfigV12 servo;
  BrakePointTuning brakePoint[3];
  BrakePointSlot brakePointSlots[3][BP_SLOT_COUNT];
  uint32_t checksum;
};

struct PersistentSettings {
  uint32_t magic;
  uint16_t version;
  AxisConfig axis[AXIS_COUNT];
  ElevatorConfig elevator;
  DrillConfig drill;
  ServoConfig servo;
  BrakePointTuning brakePoint[3];
  BrakePointSlot brakePointSlots[3][BP_SLOT_COUNT];
  uint32_t checksum;
};

enum class ServoMotion : uint8_t {
  Idle = 0,
  StateMove = 1,
  FixedMove = 2,
  Wiggle = 3
};

enum class ServoSafetyReason : uint8_t {
  Boot = 0,
  Ready = 1,
  Stop = 2,
  SafeMode = 3,
  LinkLoss = 4,
  GamepadDisconnect = 5,
  ConfigChanged = 6,
  FactoryReset = 7,
  PulseRange = 8,
  EmergencyStop = 9
};

enum class ServoError : uint8_t {
  None = 0,
  ReferenceRequired = 1,
  CurrentPulseUnknown = 2,
  TargetPulseRange = 3,
  TargetNotCalibrated = 4,
  TimerConfiguration = 5
};

struct ServoRuntime {
  bool stateInitialized = false;
  bool outputEnabled = false;
  bool pulseReferenceKnown = false;
  bool referencePulseAssumed = false;
  uint8_t currentState = 0;
  uint8_t targetState = 0;
  uint8_t referenceState = 0;
  uint16_t currentPulseUs = SERVO_CENTER_US;
  uint16_t targetPulseUs = SERVO_CENTER_US;
  uint16_t referencePulseUs = SERVO_CENTER_US;
  uint16_t homePulseUs = SERVO_CENTER_US;
  float rampPulseUs = SERVO_CENTER_US;
  ServoMotion motion = ServoMotion::Idle;
  uint32_t motionStartedMs = 0;
  uint32_t lastRampMs = 0;
  uint32_t nextStepMs = 0;
  int8_t wiggleDirection = 1;
  ServoSafetyReason safetyReason = ServoSafetyReason::Boot;
  ServoError error = ServoError::None;
};

struct PositionTarget {
  bool active = false;
  bool hold = false;
  float targetDeg = 0.0f;
};

struct ElevatorPwmRuntime {
  int16_t requested[2] = {0, 0}; // Signed PWM before sendpwm; BTS4, BTS5.
  int16_t actual[2] = {0, 0};    // Signed PWM after sendpwm; BTS4, BTS5.
};

struct AxisRuntime {
  bool angleValid = false;
  uint16_t rawAngle = 0;
  float sensorAngleDeg = 0.0f;
  float angleDeg = 0.0f;
  float previousAngleDeg = 0.0f;
  bool hasPreviousAngle = false;
  float multiTurnAngleDeg = 0.0f;
  float velocityRawDps = 0.0f;
  float velocityDps = 0.0f;
  uint32_t lastReadAttemptUs = 0;
  uint32_t nextReadAllowedUs = 0;
  uint32_t lastSampleUs = 0;
  uint16_t consecutiveReadErrors = 0;
  uint8_t activeBrakePointBoundary = BP_INVALID_BOUNDARY;
  BrakePointApproach brakePointApproach = BrakePointApproach::Inactive;
  BrakePointPhase brakePointPhase = BrakePointPhase::Inactive;
  BrakePointFaultReason brakePointFaultReason = BrakePointFaultReason::None;
  float brakePointLeadAtTriggerDeg = 0.0f;
  float brakePointRecoveryTargetDeg = 0.0f;
  Direction brakePointRecoveryDirection = Direction::Stop;
  uint32_t brakePointPhaseStartedMs = 0;
  uint8_t brakePointSettledSamples = 0;
  bool brakePointNeutralSeen = false;
  bool brakePointRetreatCommandSeen = false;
  bool brakePointRecoveryOutputAuthorized = false;
  float brakePointRecoveryLastDistanceDeg = 0.0f;
  uint32_t brakePointRecoveryLastProgressMs = 0;
  uint32_t brakePointLastDebugMs = 0;
  Direction pidOutputDirection = Direction::Stop;
  uint8_t pidOutputPwm = 0;
  uint32_t pidLastMs = 0;
  uint32_t pidReverseHoldUntilMs = 0;
  bool holdSettled = false;
};

const MotorPins motorPins[MOTOR_COUNT] = {
  {PA8,  PA9,  PB12},  // BTS1: J1 base
  {PA10, PA11, PA12},  // BTS2: J2 shoulder
  {PB7,  PB6,  PB5},   // BTS3: J3 elbow
  {PB10, PB1,  PB2},   // BTS4: J4 wrist motor 1 / elevator A
  {PB0,  PA7,  PA4},   // BTS5: J5 wrist motor 2 / elevator B
  {PA6,  PA5,  PA1}    // BTS6: J6 gripper / drill
};

PersistentSettings settings;
MotorRuntime motors[MOTOR_COUNT];
PositionTarget targets[3];
AxisRuntime axisRuntime[3];
ElevatorPwmRuntime elevatorPwmRuntime;
ServoRuntime servoRuntime;
bool drillLedStates[DRILL_LED_COUNT] = {};
bool keybindForwardActive[AXIS_COUNT] = {};
bool keybindBackwardActive[AXIS_COUNT] = {};
SystemMode mode = SystemMode::Safe;
ConfirmAction pendingConfirm = ConfirmAction::None;
bool settingsLoadedFromFlash = false;
bool settingsDirty = false;
uint8_t sensorFaultBits = 0;
uint8_t brakePointFaultBits = 0;
bool streamAS5600Active = false;
uint8_t streamAS5600Channel = 0;
uint16_t streamIntervalMs = DEFAULT_STREAM_MS;
uint32_t lastStreamMs = 0;
uint16_t statusIntervalMs = DEFAULT_STATUS_MS;
uint32_t lastStatusMs = 0;
uint32_t lastAS5600UpdateUs = 0;
uint32_t lastMotionControlUs = 0;

uint32_t bootResetFlags = 0;
uint8_t nextAS5600Axis = 0;
const char *currentAckId = nullptr;
bool hardwareWatchdogStarted = false;
uint32_t hardwareWatchdogLastReloadMs = 0;

constexpr uint8_t LOOP_HEALTH_SERIAL = 1U << 0;
constexpr uint8_t LOOP_HEALTH_SAFETY = 1U << 1;
constexpr uint8_t LOOP_HEALTH_FAST_TASKS = 1U << 2;
constexpr uint8_t LOOP_HEALTH_TELEMETRY = 1U << 3;
constexpr uint8_t LOOP_HEALTH_REQUIRED =
    LOOP_HEALTH_SERIAL | LOOP_HEALTH_SAFETY |
    LOOP_HEALTH_FAST_TASKS | LOOP_HEALTH_TELEMETRY;

enum class CpuFaultReason : uint16_t {
  HardFault = 1,
  MemManage = 2,
  BusFault = 3,
  UsageFault = 4
};

constexpr uint32_t CPU_FAULT_RECORD_MAGIC = 0x4641554CUL; // "FAUL"
constexpr uint16_t CPU_FAULT_RECORD_VERSION = 1;
constexpr uint8_t CPU_FAULT_RECORD_WORDS = 18;
constexpr uint8_t CPU_FAULT_CHECKSUM_INDEX = 17;
constexpr uint8_t BOOT_STREAK_MAGIC_INDEX = 18;
constexpr uint8_t BOOT_STREAK_COUNT_INDEX = 19;
constexpr uint32_t BOOT_STREAK_MAGIC = 0x42535452UL; // "BSTR"
constexpr uint32_t BOOT_STREAK_CLEAR_AFTER_MS = 10000UL;
uint32_t bootResetStreak = 0;
bool bootResetStreakCleared = false;

volatile uint32_t *faultBackupRegisters() {
  return &RTC->BKP0R;
}

void enableBackupRegisterWrites() {
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  (void)RCC->APB1ENR;
  PWR->CR |= PWR_CR_DBP;
  __DSB();
}

uint32_t faultRecordChecksum(const volatile uint32_t *record) {
  uint32_t hash = 2166136261UL;
  for (uint8_t i = 1; i < CPU_FAULT_CHECKSUM_INDEX; ++i) {
    hash ^= record[i];
    hash *= 16777619UL;
  }
  return hash;
}

extern "C" __attribute__((noreturn, noinline, used))
void faultCaptureAndReset(uint32_t *exceptionStack,
                          uint32_t exceptionReturn,
                          uint32_t reasonValue) {
  __disable_irq();
  enableBackupRegisterWrites();
  volatile uint32_t *record = faultBackupRegisters();

  // Invalidate first so a reset during record creation cannot look valid.
  record[0] = 0;
  record[1] =
      (static_cast<uint32_t>(CPU_FAULT_RECORD_VERSION) << 16) |
      (reasonValue & 0xFFFFUL);

  uint32_t *coreFrame = exceptionStack;
  if ((exceptionReturn & (1UL << 4)) == 0U) {
    // The extended floating-point frame precedes the basic eight-word frame.
    coreFrame += 18;
  }
  const uintptr_t frameAddress = reinterpret_cast<uintptr_t>(coreFrame);
  const bool validFrame =
      frameAddress >= 0x20000000UL &&
      frameAddress <= (0x20018000UL - 8UL * sizeof(uint32_t));

  record[2] = validFrame ? coreFrame[6] : 0; // PC
  record[3] = validFrame ? coreFrame[5] : 0; // LR
  record[4] = validFrame ? coreFrame[7] : 0; // xPSR
  record[5] = exceptionReturn;
  record[6] = SCB->CFSR;
  record[7] = SCB->HFSR;
  record[8] = SCB->DFSR;
  record[9] = SCB->AFSR;
  record[10] = SCB->MMFAR;
  record[11] = SCB->BFAR;
  record[12] = validFrame ? coreFrame[0] : 0; // R0
  record[13] = validFrame ? coreFrame[1] : 0; // R1
  record[14] = validFrame ? coreFrame[2] : 0; // R2
  record[15] = validFrame ? coreFrame[3] : 0; // R3
  record[16] = validFrame ? coreFrame[4] : 0; // R12
  record[CPU_FAULT_CHECKSUM_INDEX] = faultRecordChecksum(record);
  __DSB();
  record[0] = CPU_FAULT_RECORD_MAGIC;
  __DSB();

  NVIC_SystemReset();
  while (true) {
  }
}

extern "C" __attribute__((naked, noreturn, used))
void HardFault_Handler() {
  __asm volatile(
      "tst lr, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "mov r1, lr\n"
      "movs r2, #1\n"
      "b faultCaptureAndReset\n");
}

extern "C" __attribute__((naked, noreturn, used))
void MemManage_Handler() {
  __asm volatile(
      "tst lr, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "mov r1, lr\n"
      "movs r2, #2\n"
      "b faultCaptureAndReset\n");
}

extern "C" __attribute__((naked, noreturn, used))
void BusFault_Handler() {
  __asm volatile(
      "tst lr, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "mov r1, lr\n"
      "movs r2, #3\n"
      "b faultCaptureAndReset\n");
}

extern "C" __attribute__((naked, noreturn, used))
void UsageFault_Handler() {
  __asm volatile(
      "tst lr, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "mov r1, lr\n"
      "movs r2, #4\n"
      "b faultCaptureAndReset\n");
}

// Debugger-watch counters. Rate and dt-window values are refreshed once per second.
uint32_t as5600_update_count = 0;
uint32_t telemetry_send_count = 0;
uint32_t as5600_error_count = 0;
uint32_t i2c_error_count = 0;
uint32_t i2c_recovery_count = 0;
uint32_t as5600_last_dt_us = 0;
uint32_t as5600_min_dt_us = 0;
uint32_t as5600_max_dt_us = 0;
uint32_t as5600_avg_dt_us = 0;
uint32_t as5600_updates_per_second = 0;
uint32_t telemetry_sends_per_second = 0;
uint32_t as5600_errors_per_second = 0;
uint8_t as5600_last_axis = 0;
float as5600_velocity_raw_dps = 0.0f;
float as5600_velocity_filtered_dps = 0.0f;

uint32_t microClockCyclesPerUs = 1;
uint32_t microClockLastCycles = 0;
uint32_t microClockUs = 0;
uint32_t microClockRemainderCycles = 0;
uint32_t debugWindowStartMs = 0;
uint32_t debugWindowUpdateStart = 0;
uint32_t debugWindowTelemetryStart = 0;
uint32_t debugWindowErrorStart = 0;
uint32_t debugWindowDtCount = 0;
uint32_t debugWindowDtTotalUs = 0;
uint32_t debugWindowDtMinUs = UINT32_MAX;
uint32_t debugWindowDtMaxUs = 0;

char rxLine[128];
uint8_t rxLen = 0;

const char *modeName(SystemMode value) {
  switch (value) {
    case SystemMode::Arm: return "ARM";
    case SystemMode::Drill: return "DRILL";
    case SystemMode::ArmTest: return "ARMTEST";
    default: return "SAFE";
  }
}

const char *modeNameLower(SystemMode value) {
  switch (value) {
    case SystemMode::Arm: return "arm";
    case SystemMode::Drill: return "drill";
    case SystemMode::ArmTest: return "armtest";
    default: return "safe";
  }
}

const char *directionName(Direction value) {
  switch (value) {
    case Direction::Forward: return "FORWARD";
    case Direction::Backward: return "BACKWARD";
    default: return "STOP";
  }
}

const char *directionNameLower(Direction value) {
  switch (value) {
    case Direction::Forward: return "forward";
    case Direction::Backward: return "backward";
    default: return "stop";
  }
}

const char *stopModeName(uint8_t value) {
  switch (static_cast<StopMode>(value)) {
    case StopMode::Brake: return "BRAKE";
    case StopMode::Hold: return "HOLD";
    case StopMode::Hybrid: return "HYBRID";
    default: return "COAST";
  }
}

const char *telemetryStopModeName() {
  bool anyBraking = false;
  bool anyHolding = false;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    anyBraking = anyBraking || motors[i].braking;
  }
  for (uint8_t i = 0; i < 3; ++i) {
    anyHolding = anyHolding || (targets[i].active && targets[i].hold);
  }
  if (anyBraking && anyHolding) {
    return "hybrid";
  }
  if (anyBraking) {
    return "brake";
  }
  if (anyHolding) {
    return "hybrid";
  }
  return "coast";
}

const char *as5600MagnetStatusName(uint8_t status) {
  const bool magnetDetected = (status & 0x20) != 0;
  const bool magnetTooWeak = (status & 0x10) != 0;
  const bool magnetTooStrong = (status & 0x08) != 0;
  if (!magnetDetected) {
    return "NO_MAGNET";
  }
  if (magnetTooWeak) {
    return "MAGNET_WEAK";
  }
  if (magnetTooStrong) {
    return "MAGNET_STRONG";
  }
  return "MAGNET_OK";
}

uint32_t checksumBytes(const uint8_t *data, size_t len) {
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < len; ++i) {
    hash ^= data[i];
    hash *= 16777619UL;
  }
  return hash;
}

uint32_t settingsChecksum(const PersistentSettings &value) {
  return checksumBytes(reinterpret_cast<const uint8_t *>(&value),
                       sizeof(PersistentSettings) - sizeof(value.checksum));
}

uint32_t settingsChecksumV5(const PersistentSettingsV5 &value) {
  return checksumBytes(reinterpret_cast<const uint8_t *>(&value),
                       sizeof(PersistentSettingsV5) - sizeof(value.checksum));
}

uint32_t settingsChecksumV6(const PersistentSettingsV6 &value) {
  return checksumBytes(reinterpret_cast<const uint8_t *>(&value),
                       sizeof(PersistentSettingsV6) - sizeof(value.checksum));
}

uint32_t settingsChecksumV7(const PersistentSettingsV7 &value) {
  return checksumBytes(reinterpret_cast<const uint8_t *>(&value),
                       sizeof(PersistentSettingsV7) - sizeof(value.checksum));
}

uint32_t settingsChecksumV8(const PersistentSettingsV8 &value) {
  return checksumBytes(reinterpret_cast<const uint8_t *>(&value),
                       sizeof(PersistentSettingsV8) - sizeof(value.checksum));
}

uint32_t settingsChecksumV9(const PersistentSettingsV9 &value) {
  return checksumBytes(reinterpret_cast<const uint8_t *>(&value),
                       sizeof(PersistentSettingsV9) - sizeof(value.checksum));
}

uint32_t settingsChecksumV10(const PersistentSettingsV10 &value) {
  return checksumBytes(reinterpret_cast<const uint8_t *>(&value),
                       sizeof(PersistentSettingsV10) - sizeof(value.checksum));
}

uint32_t settingsChecksumV11(const PersistentSettingsV11 &value) {
  return checksumBytes(reinterpret_cast<const uint8_t *>(&value),
                       sizeof(PersistentSettingsV11) - sizeof(value.checksum));
}

uint32_t settingsChecksumV12(const PersistentSettingsV12 &value) {
  return checksumBytes(reinterpret_cast<const uint8_t *>(&value),
                       sizeof(PersistentSettingsV12) - sizeof(value.checksum));
}

float normalize360(float angle) {
  float result = fmodf(angle, 360.0f);
  if (result < 0.0f) {
    result += 360.0f;
  }
  return result;
}

float shortestErrorDeg(float target, float current) {
  float error = normalize360(target) - normalize360(current);
  if (error > 180.0f) {
    error -= 360.0f;
  } else if (error < -180.0f) {
    error += 360.0f;
  }
  return error;
}

void initializeMicrosecondClock() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  microClockCyclesPerUs = SystemCoreClock / 1000000UL;
  if (microClockCyclesPerUs == 0) {
    microClockCyclesPerUs = 1;
  }
  microClockLastCycles = DWT->CYCCNT;
  microClockUs = 0;
  microClockRemainderCycles = 0;
}

uint32_t GetMicroseconds() {
  const uint32_t currentCycles = DWT->CYCCNT;
  const uint32_t elapsedCycles = currentCycles - microClockLastCycles;
  microClockLastCycles = currentCycles;

  microClockUs += elapsedCycles / microClockCyclesPerUs;
  microClockRemainderCycles += elapsedCycles % microClockCyclesPerUs;
  if (microClockRemainderCycles >= microClockCyclesPerUs) {
    microClockUs += microClockRemainderCycles / microClockCyclesPerUs;
    microClockRemainderCycles %= microClockCyclesPerUs;
  }
  return microClockUs;
}

void recordAS5600SampleTiming(uint32_t dtUs) {
  as5600_last_dt_us = dtUs;
  if (dtUs < debugWindowDtMinUs) {
    debugWindowDtMinUs = dtUs;
  }
  if (dtUs > debugWindowDtMaxUs) {
    debugWindowDtMaxUs = dtUs;
  }
  debugWindowDtTotalUs += dtUs;
  ++debugWindowDtCount;
}

void serviceDebugCounters() {
  const uint32_t nowMs = millis();
  const uint32_t elapsedMs = nowMs - debugWindowStartMs;
  if (elapsedMs < 1000) {
    return;
  }

  as5600_updates_per_second =
      ((as5600_update_count - debugWindowUpdateStart) * 1000UL) / elapsedMs;
  telemetry_sends_per_second =
      ((telemetry_send_count - debugWindowTelemetryStart) * 1000UL) / elapsedMs;
  as5600_errors_per_second =
      ((as5600_error_count - debugWindowErrorStart) * 1000UL) / elapsedMs;
  as5600_min_dt_us = debugWindowDtCount == 0 ? 0 : debugWindowDtMinUs;
  as5600_max_dt_us = debugWindowDtMaxUs;
  as5600_avg_dt_us = debugWindowDtCount == 0 ? 0 : debugWindowDtTotalUs / debugWindowDtCount;

  debugWindowStartMs = nowMs;
  debugWindowUpdateStart = as5600_update_count;
  debugWindowTelemetryStart = telemetry_send_count;
  debugWindowErrorStart = as5600_error_count;
  debugWindowDtCount = 0;
  debugWindowDtTotalUs = 0;
  debugWindowDtMinUs = UINT32_MAX;
  debugWindowDtMaxUs = 0;
}

uint8_t clampPwm(int value, uint8_t maxAllowed) {
  if (value < 0) {
    return 0;
  }
  if (value > maxAllowed) {
    return maxAllowed;
  }
  return static_cast<uint8_t>(value);
}

constexpr uint8_t scalePwmByPercent(uint8_t pwm, uint8_t percent) {
  return static_cast<uint8_t>(
      (static_cast<uint16_t>(pwm) * percent) / SENDPWM_PERCENT_MAX);
}

static_assert(scalePwmByPercent(255, 100) == 255,
              "1.00 sendpwm must preserve PWM");
static_assert(scalePwmByPercent(255, 75) == 191,
              "sendpwm must floor fractional PWM");
static_assert(scalePwmByPercent(1, 75) == 0,
              "sendpwm must floor toward zero for PWM magnitude");

void sendLine(const char *text) {
  LinkSerial.println(text);
}

void sendOk(const char *text) {
  if (currentAckId != nullptr) {
    LinkSerial.print("ACK ");
    LinkSerial.print(currentAckId);
    LinkSerial.print(' ');
  }
  LinkSerial.print("OK ");
  LinkSerial.println(text);
}

void sendErr(const char *text) {
  if (currentAckId != nullptr) {
    LinkSerial.print("ACK ");
    LinkSerial.print(currentAckId);
    LinkSerial.print(' ');
  }
  LinkSerial.print("ERR ");
  LinkSerial.println(text);
}

void setDefaults() {
  memset(&settings, 0, sizeof(settings));
  settings.magic = SETTINGS_MAGIC;
  settings.version = SETTINGS_VERSION;

  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    settings.axis[i].invert = 0;
    settings.axis[i].maxPwm = MAX_PWM;
    settings.axis[i].defaultPwm = DEFAULT_PWM;
    settings.axis[i].posGain = DEFAULT_GAIN;
    settings.axis[i].negGain = DEFAULT_GAIN;
    settings.axis[i].dead = DEFAULT_DEAD;
    settings.axis[i].stopMode = static_cast<uint8_t>(StopMode::Coast);
    settings.axis[i].kp = DEFAULT_KP;
    settings.axis[i].kd = DEFAULT_KD;
    settings.axis[i].minPwm = DEFAULT_MIN_PWM;
    settings.axis[i].toleranceDeg = POSITION_TOLERANCE_DEG;
    settings.axis[i].zeroOffsetDeg = 0.0f;
    settings.axis[i].keybindForward = KEYBIND_DISABLED;
    settings.axis[i].keybindBackward = KEYBIND_DISABLED;
    settings.axis[i].as5600Channel = i < 3 ? i : AS5600_CHANNEL_DISABLED;
  }

  for (uint8_t i = 0; i < 3; ++i) {
    settings.brakePoint[i].reactionMs = DEFAULT_BP_REACTION_MS;
    settings.brakePoint[i].decelDps2 = DEFAULT_BP_DECEL_DPS2[i];
    settings.brakePoint[i].leadMarginDeg = DEFAULT_BP_LEAD_MARGIN_DEG[i];
    settings.brakePoint[i].recoveryClearanceDeg = DEFAULT_BP_RECOVERY_CLEARANCE_DEG[i];
    settings.brakePoint[i].recoveryMaxPwm = DEFAULT_BP_RECOVERY_PWM[i];
    settings.brakePoint[i].recoveryTimeoutMs = DEFAULT_BP_RECOVERY_TIMEOUT_MS[i];
    for (uint8_t slot = 0; slot < BP_SLOT_COUNT; ++slot) {
      settings.brakePointSlots[i][slot].angleDeg = 0.0f;
      settings.brakePointSlots[i][slot].mode =
          static_cast<uint8_t>(BrakePointMode::Disabled);
    }
  }

  settings.axis[3].keybindForward = DEFAULT_HAT0_UP_ID;      // J4 pitch up
  settings.axis[3].keybindBackward = DEFAULT_HAT0_DOWN_ID;    // J4 pitch down
  settings.axis[5].keybindForward = DEFAULT_HAT0_RIGHT_ID;    // J6 gripper right
  settings.axis[5].keybindBackward = DEFAULT_HAT0_LEFT_ID;    // J6 gripper left

  settings.elevator = {
      0,
      MAX_PWM,
      DEFAULT_PWM,
      {SENDPWM_PERCENT_MAX, SENDPWM_PERCENT_MAX}};
  settings.drill = {0, MAX_PWM, DEFAULT_PWM};
  settings.servo = {
      DEFAULT_SERVO_STATE_ANGLE_DEG,
      DEFAULT_SERVO_WIGGLE_ANGLE_DEG,
      0,
      KEYBIND_DISABLED,
      KEYBIND_DISABLED,
      DEFAULT_SERVO_SPEED_DPS};
  settings.checksum = settingsChecksum(settings);
}

void copyLegacyAxisConfig(AxisConfig &destination,
                          const LegacyAxisConfigV6 &source) {
  destination.invert = source.invert;
  destination.maxPwm = source.maxPwm;
  destination.defaultPwm = source.defaultPwm;
  destination.posGain = source.posGain;
  destination.negGain = source.negGain;
  destination.dead = source.dead;
  destination.stopMode = source.stopMode;
  destination.kp = source.kp;
  destination.kd = source.kd;
  destination.minPwm = source.minPwm;
  destination.toleranceDeg = source.toleranceDeg;
  destination.zeroOffsetDeg = source.zeroOffsetDeg;
  destination.keybindForward = source.keybindForward;
  destination.keybindBackward = source.keybindBackward;
  destination.as5600Channel = source.as5600Channel;
}

void copyLegacyElevatorConfig(ElevatorConfig &destination,
                              const DrillConfig &source) {
  destination.invert = source.invert;
  destination.maxPwm = source.maxPwm;
  destination.defaultPwm = source.defaultPwm;
  destination.sendPwmPercent[0] = SENDPWM_PERCENT_MAX;
  destination.sendPwmPercent[1] = SENDPWM_PERCENT_MAX;
}

void migrateLegacyBrakePoints(
    const LegacyAxisConfigV6 legacyAxis[AXIS_COUNT]) {
  for (uint8_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
    const LegacyAxisConfigV6 &legacy = legacyAxis[axisIndex];
    if (legacy.brakePointEnabled) {
      settings.brakePointSlots[axisIndex][0].angleDeg =
          normalize360(legacy.brakePointDeg);
      settings.brakePointSlots[axisIndex][0].mode =
          static_cast<uint8_t>(BrakePointMode::Wall);
    }
  }
}

void markSettingsDirty() {
  settings.magic = SETTINGS_MAGIC;
  settings.version = SETTINGS_VERSION;
  settings.checksum = settingsChecksum(settings);
  settingsDirty = true;
}

bool servoStoredCalibrationValid(const ServoConfig &servo) {
  if ((servo.statePulseValidMask &
       static_cast<uint8_t>(~SERVO_STATE_VALID_MASK)) != 0U) {
    return false;
  }
  for (uint8_t i = 0; i < SERVO_STATE_COUNT; ++i) {
    if ((servo.statePulseValidMask & (1U << i)) == 0U) {
      continue;
    }
    if (servo.statePulseUs[i] < SERVO_MIN_US ||
        servo.statePulseUs[i] > SERVO_MAX_US) {
      return false;
    }
    for (uint8_t j = i + 1; j < SERVO_STATE_COUNT; ++j) {
      if ((servo.statePulseValidMask & (1U << j)) == 0U) {
        continue;
      }
      const bool ordered =
          servo.invert ? servo.statePulseUs[i] > servo.statePulseUs[j]
                       : servo.statePulseUs[i] < servo.statePulseUs[j];
      if (!ordered) {
        return false;
      }
    }
  }
  return true;
}

bool persistSettingsToFlash() {
  settings.magic = SETTINGS_MAGIC;
  settings.version = SETTINGS_VERSION;
  settings.checksum = settingsChecksum(settings);

#if !defined(DATA_EEPROM_BASE)
  eeprom_buffer_fill();
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&settings);
  for (uint16_t i = 0; i < sizeof(settings); ++i) {
    eeprom_buffered_write_byte(i, bytes[i]);
  }
  eeprom_buffer_flush();
#else
  EEPROM.put(0, settings);
#endif

  settingsDirty = false;
  settingsLoadedFromFlash = true;
  return true;
}

void loadSettings() {
  EEPROM.get(0, settings);
  const bool valid = settings.magic == SETTINGS_MAGIC &&
                      settings.version == SETTINGS_VERSION &&
                      settings.checksum == settingsChecksum(settings);
  if (valid) {
    settingsLoadedFromFlash = true;
    settingsDirty = false;
    if (settings.servo.speedDegPerSec < MIN_SERVO_SPEED_DPS ||
        settings.servo.speedDegPerSec > MAX_SERVO_SPEED_DPS) {
      settings.servo.speedDegPerSec = DEFAULT_SERVO_SPEED_DPS;
      markSettingsDirty();
      settingsLoadedFromFlash = true;
    }
    if (!servoStoredCalibrationValid(settings.servo)) {
      memset(settings.servo.statePulseUs,
             0,
             sizeof(settings.servo.statePulseUs));
      settings.servo.statePulseValidMask = 0;
      markSettingsDirty();
      settingsLoadedFromFlash = true;
    }
    if (settings.servo.keybindNext != KEYBIND_DISABLED ||
        settings.servo.keybindPrevious != KEYBIND_DISABLED) {
      settings.servo.keybindNext = KEYBIND_DISABLED;
      settings.servo.keybindPrevious = KEYBIND_DISABLED;
      markSettingsDirty();
      settingsLoadedFromFlash = true;
    }
    return;
  }

  PersistentSettingsV12 legacyV12 = {};
  EEPROM.get(0, legacyV12);
  const bool legacyV12Valid =
      legacyV12.magic == SETTINGS_MAGIC &&
      legacyV12.version == 12 &&
      legacyV12.checksum == settingsChecksumV12(legacyV12);
  if (legacyV12Valid) {
    setDefaults();
    memcpy(settings.axis, legacyV12.axis, sizeof(settings.axis));
    settings.elevator = legacyV12.elevator;
    settings.drill = legacyV12.drill;
    settings.servo.stateAngleDeg = legacyV12.servo.stateAngleDeg;
    settings.servo.wiggleAngleDeg = legacyV12.servo.wiggleAngleDeg;
    settings.servo.invert = legacyV12.servo.invert;
    settings.servo.keybindNext = KEYBIND_DISABLED;
    settings.servo.keybindPrevious = KEYBIND_DISABLED;
    settings.servo.speedDegPerSec = legacyV12.servo.speedDegPerSec;
    memcpy(settings.servo.statePulseUs,
           legacyV12.servo.statePulseUs,
           sizeof(legacyV12.servo.statePulseUs));
    settings.servo.statePulseValidMask =
        legacyV12.servo.statePulseValidMask & 0x0FU;
    memcpy(settings.brakePoint,
           legacyV12.brakePoint,
           sizeof(settings.brakePoint));
    memcpy(settings.brakePointSlots,
           legacyV12.brakePointSlots,
           sizeof(settings.brakePointSlots));
    markSettingsDirty();
    settingsLoadedFromFlash = true;
    return;
  }

  PersistentSettingsV11 legacyV11 = {};
  EEPROM.get(0, legacyV11);
  const bool legacyV11Valid =
      legacyV11.magic == SETTINGS_MAGIC &&
      legacyV11.version == 11 &&
      legacyV11.checksum == settingsChecksumV11(legacyV11);
  if (legacyV11Valid) {
    setDefaults();
    memcpy(settings.axis, legacyV11.axis, sizeof(settings.axis));
    settings.elevator = legacyV11.elevator;
    settings.drill = legacyV11.drill;
    settings.servo.stateAngleDeg = legacyV11.servo.stateAngleDeg;
    settings.servo.wiggleAngleDeg = legacyV11.servo.wiggleAngleDeg;
    settings.servo.invert = legacyV11.servo.invert;
    settings.servo.keybindNext = KEYBIND_DISABLED;
    settings.servo.keybindPrevious = KEYBIND_DISABLED;
    settings.servo.speedDegPerSec = legacyV11.servo.speedDegPerSec;
    memcpy(settings.brakePoint,
           legacyV11.brakePoint,
           sizeof(settings.brakePoint));
    memcpy(settings.brakePointSlots,
           legacyV11.brakePointSlots,
           sizeof(settings.brakePointSlots));
    markSettingsDirty();
    settingsLoadedFromFlash = true;
    return;
  }

  PersistentSettingsV10 legacyV10 = {};
  EEPROM.get(0, legacyV10);
  const bool legacyV10Valid =
      legacyV10.magic == SETTINGS_MAGIC &&
      legacyV10.version == 10 &&
      legacyV10.checksum == settingsChecksumV10(legacyV10);
  if (legacyV10Valid) {
    setDefaults();
    memcpy(settings.axis, legacyV10.axis, sizeof(settings.axis));
    settings.elevator = legacyV10.elevator;
    settings.drill = legacyV10.drill;
    settings.servo.stateAngleDeg = legacyV10.servo.stateAngleDeg;
    settings.servo.wiggleAngleDeg = legacyV10.servo.wiggleAngleDeg;
    settings.servo.invert = legacyV10.servo.invert;
    settings.servo.keybindNext = KEYBIND_DISABLED;
    settings.servo.keybindPrevious = KEYBIND_DISABLED;
    memcpy(settings.brakePoint,
           legacyV10.brakePoint,
           sizeof(settings.brakePoint));
    memcpy(settings.brakePointSlots,
           legacyV10.brakePointSlots,
           sizeof(settings.brakePointSlots));
    markSettingsDirty();
    settingsLoadedFromFlash = true;
    return;
  }

  PersistentSettingsV9 legacyV9 = {};
  EEPROM.get(0, legacyV9);
  const bool legacyV9Valid =
      legacyV9.magic == SETTINGS_MAGIC &&
      legacyV9.version == 9 &&
      legacyV9.checksum == settingsChecksumV9(legacyV9);
  if (legacyV9Valid) {
    setDefaults();
    memcpy(settings.axis, legacyV9.axis, sizeof(settings.axis));
    settings.elevator = legacyV9.elevator;
    settings.drill = legacyV9.drill;
    memcpy(settings.brakePoint,
           legacyV9.brakePoint,
           sizeof(settings.brakePoint));
    memcpy(settings.brakePointSlots,
           legacyV9.brakePointSlots,
           sizeof(settings.brakePointSlots));
    markSettingsDirty();
    settingsLoadedFromFlash = true;
    return;
  }

  PersistentSettingsV8 legacyV8 = {};
  EEPROM.get(0, legacyV8);
  const bool legacyV8Valid =
      legacyV8.magic == SETTINGS_MAGIC &&
      legacyV8.version == 8 &&
      legacyV8.checksum == settingsChecksumV8(legacyV8);
  if (legacyV8Valid) {
    setDefaults();
    memcpy(settings.axis, legacyV8.axis, sizeof(settings.axis));
    copyLegacyElevatorConfig(settings.elevator, legacyV8.elevator);
    settings.drill = legacyV8.drill;
    memcpy(settings.brakePoint,
           legacyV8.brakePoint,
           sizeof(settings.brakePoint));
    memcpy(settings.brakePointSlots,
           legacyV8.brakePointSlots,
           sizeof(settings.brakePointSlots));
    markSettingsDirty();
    settingsLoadedFromFlash = true;
    return;
  }

  PersistentSettingsV7 legacyV7 = {};
  EEPROM.get(0, legacyV7);
  const bool legacyV7Valid =
      legacyV7.magic == SETTINGS_MAGIC &&
      legacyV7.version == 7 &&
      legacyV7.checksum == settingsChecksumV7(legacyV7);
  if (legacyV7Valid) {
    setDefaults();
    memcpy(settings.axis, legacyV7.axis, sizeof(settings.axis));
    copyLegacyElevatorConfig(settings.elevator, legacyV7.elevator);
    settings.drill = legacyV7.drill;
    memcpy(settings.brakePoint,
           legacyV7.brakePoint,
           sizeof(settings.brakePoint));
    memcpy(settings.brakePointSlots,
           legacyV7.brakePointSlots,
           sizeof(settings.brakePointSlots));
    markSettingsDirty();
    settingsLoadedFromFlash = true;
    return;
  }

  PersistentSettingsV6 legacyV6 = {};
  EEPROM.get(0, legacyV6);
  const bool legacyV6Valid =
      legacyV6.magic == SETTINGS_MAGIC &&
      legacyV6.version == 6 &&
      legacyV6.checksum == settingsChecksumV6(legacyV6);
  if (legacyV6Valid) {
    setDefaults();
    for (uint8_t axisIndex = 0; axisIndex < AXIS_COUNT; ++axisIndex) {
      copyLegacyAxisConfig(settings.axis[axisIndex],
                           legacyV6.axis[axisIndex]);
    }
    copyLegacyElevatorConfig(settings.elevator, legacyV6.elevator);
    settings.drill = legacyV6.drill;
    memcpy(settings.brakePoint,
           legacyV6.brakePoint,
           sizeof(legacyV6.brakePoint));
    migrateLegacyBrakePoints(legacyV6.axis);
    markSettingsDirty();
    settingsLoadedFromFlash = true;
    return;
  }

  PersistentSettingsV5 legacy = {};
  EEPROM.get(0, legacy);
  const bool legacyValid = legacy.magic == SETTINGS_MAGIC &&
                           legacy.version == 5 &&
                           legacy.checksum == settingsChecksumV5(legacy);
  if (legacyValid) {
    setDefaults();
    for (uint8_t axisIndex = 0; axisIndex < AXIS_COUNT; ++axisIndex) {
      copyLegacyAxisConfig(settings.axis[axisIndex],
                           legacy.axis[axisIndex]);
    }
    copyLegacyElevatorConfig(settings.elevator, legacy.elevator);
    settings.drill = legacy.drill;
    migrateLegacyBrakePoints(legacy.axis);
    markSettingsDirty();
    settingsLoadedFromFlash = true;
    return;
  }

  setDefaults();
  settingsLoadedFromFlash = false;
  settingsDirty = true;
}

void forceMotorPinLow(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void configureDrillAccessoryPins() {
  // Preload the active-LOW relay outputs HIGH before enabling the output
  // drivers, minimizing the reset-to-setup interval in which a relay can pull in.
  for (uint8_t i = 0; i < DRILL_LED_COUNT; ++i) {
    digitalWrite(DRILL_LED_PINS[i], HIGH);
    pinMode(DRILL_LED_PINS[i], OUTPUT);
    drillLedStates[i] = false;
  }
  // Hold the servo signal deterministically LOW whenever TIM5 is inactive.
  // PWM is enabled only after setup by an explicit valid servo command.
  digitalWrite(SERVO_PIN, LOW);
  pinMode(SERVO_PIN, OUTPUT);
}

void setDrillLed(uint8_t index, bool on) {
  if (index >= DRILL_LED_COUNT) {
    return;
  }
  drillLedStates[index] = on;
  digitalWrite(DRILL_LED_PINS[index], on ? LOW : HIGH);
}

float servoAngleForPulse(float pulseUs) {
  if (pulseUs <= static_cast<float>(SERVO_STANDARD_MIN_US)) {
    return -SERVO_MAX_ABSOLUTE_ANGLE_DEG +
           (pulseUs - static_cast<float>(SERVO_MIN_US)) *
               (SERVO_MAX_ABSOLUTE_ANGLE_DEG -
                SERVO_STANDARD_HALF_TRAVEL_DEG) /
               static_cast<float>(SERVO_STANDARD_MIN_US - SERVO_MIN_US);
  }
  if (pulseUs <= static_cast<float>(SERVO_STANDARD_MAX_US)) {
    return -SERVO_STANDARD_HALF_TRAVEL_DEG +
           (pulseUs - static_cast<float>(SERVO_STANDARD_MIN_US)) *
               (2.0f * SERVO_STANDARD_HALF_TRAVEL_DEG) /
               static_cast<float>(SERVO_STANDARD_MAX_US -
                                  SERVO_STANDARD_MIN_US);
  }
  return SERVO_STANDARD_HALF_TRAVEL_DEG +
         (pulseUs - static_cast<float>(SERVO_STANDARD_MAX_US)) *
             (SERVO_MAX_ABSOLUTE_ANGLE_DEG -
              SERVO_STANDARD_HALF_TRAVEL_DEG) /
             static_cast<float>(SERVO_MAX_US - SERVO_STANDARD_MAX_US);
}

float servoPulseForAngle(float angleDeg) {
  if (angleDeg <= -SERVO_STANDARD_HALF_TRAVEL_DEG) {
    return static_cast<float>(SERVO_MIN_US) +
           (angleDeg + SERVO_MAX_ABSOLUTE_ANGLE_DEG) *
               static_cast<float>(SERVO_STANDARD_MIN_US - SERVO_MIN_US) /
               (SERVO_MAX_ABSOLUTE_ANGLE_DEG -
                SERVO_STANDARD_HALF_TRAVEL_DEG);
  }
  if (angleDeg <= SERVO_STANDARD_HALF_TRAVEL_DEG) {
    return static_cast<float>(SERVO_STANDARD_MIN_US) +
           (angleDeg + SERVO_STANDARD_HALF_TRAVEL_DEG) *
               static_cast<float>(SERVO_STANDARD_MAX_US -
                                  SERVO_STANDARD_MIN_US) /
               (2.0f * SERVO_STANDARD_HALF_TRAVEL_DEG);
  }
  return static_cast<float>(SERVO_STANDARD_MAX_US) +
         (angleDeg - SERVO_STANDARD_HALF_TRAVEL_DEG) *
             static_cast<float>(SERVO_MAX_US - SERVO_STANDARD_MAX_US) /
             (SERVO_MAX_ABSOLUTE_ANGLE_DEG -
              SERVO_STANDARD_HALF_TRAVEL_DEG);
}

bool servoPulseForAngleChecked(float angleDeg, uint16_t &pulseUs) {
  if (angleDeg < -SERVO_MAX_ABSOLUTE_ANGLE_DEG ||
      angleDeg > SERVO_MAX_ABSOLUTE_ANGLE_DEG) {
    return false;
  }
  const long pulse = lroundf(servoPulseForAngle(angleDeg));
  if (pulse < SERVO_MIN_US || pulse > SERVO_MAX_US) {
    return false;
  }
  pulseUs = static_cast<uint16_t>(pulse);
  return true;
}

const char *servoSafetyReasonName(ServoSafetyReason reason) {
  switch (reason) {
    case ServoSafetyReason::Ready:
      return "READY";
    case ServoSafetyReason::Stop:
      return "STOP";
    case ServoSafetyReason::SafeMode:
      return "SAFE_MODE";
    case ServoSafetyReason::LinkLoss:
      return "LINK_LOSS";
    case ServoSafetyReason::GamepadDisconnect:
      return "GAMEPAD_DISCONNECT";
    case ServoSafetyReason::ConfigChanged:
      return "CONFIG_CHANGED";
    case ServoSafetyReason::FactoryReset:
      return "FACTORY_RESET";
    case ServoSafetyReason::PulseRange:
      return "PULSE_RANGE";
    case ServoSafetyReason::EmergencyStop:
      return "ESTOP";
    case ServoSafetyReason::Boot:
    default:
      return "BOOT";
  }
}

const char *servoErrorName(ServoError error) {
  switch (error) {
    case ServoError::ReferenceRequired:
      return "REFERENCE_REQUIRED";
    case ServoError::CurrentPulseUnknown:
      return "CURRENT_PULSE_UNKNOWN";
    case ServoError::TargetPulseRange:
      return "TARGET_PULSE_RANGE";
    case ServoError::TargetNotCalibrated:
      return "TARGET_NOT_CALIBRATED";
    case ServoError::TimerConfiguration:
      return "TIMER_CONFIGURATION";
    case ServoError::None:
    default:
      return "NONE";
  }
}

const char *servoStateName(uint8_t state) {
  switch (state) {
    case 1:
      return "FINAL_LID";
    case 2:
      return "TUBE_4";
    case 3:
      return "TUBE_3";
    case 4:
      return "TUBE_2";
    case 5:
      return "TUBE_1";
    default:
      return "NA";
  }
}

bool servoPulseForAnchor(uint16_t referencePulseUs,
                         uint8_t referenceState,
                         uint8_t state,
                         uint16_t &pulseUs) {
  if (referenceState < SERVO_STATE_MIN ||
      referenceState > SERVO_STATE_MAX ||
      state < SERVO_STATE_MIN ||
      state > SERVO_STATE_MAX) {
    return false;
  }
  const int stateDelta =
      static_cast<int>(state) - static_cast<int>(referenceState);
  const float referenceAngleDeg =
      servoAngleForPulse(static_cast<float>(referencePulseUs));
  const float targetAngleDeg =
      referenceAngleDeg +
      static_cast<float>(stateDelta) * settings.servo.stateAngleDeg *
          (settings.servo.invert ? -1.0f : 1.0f);
  return servoPulseForAngleChecked(targetAngleDeg, pulseUs);
}

bool servoStatePulseCalibrated(uint8_t state) {
  return state >= SERVO_STATE_MIN &&
         state <= SERVO_STATE_MAX &&
         (settings.servo.statePulseValidMask &
          (1U << (state - SERVO_STATE_MIN))) != 0U;
}

bool servoCalibrationPulseFits(uint8_t state, uint16_t pulseUs) {
  if (state < SERVO_STATE_MIN || state > SERVO_STATE_MAX ||
      pulseUs < SERVO_MIN_US || pulseUs > SERVO_MAX_US) {
    return false;
  }
  for (uint8_t otherState = SERVO_STATE_MIN;
       otherState <= SERVO_STATE_MAX;
       ++otherState) {
    if (otherState == state || !servoStatePulseCalibrated(otherState)) {
      continue;
    }
    const uint16_t otherPulse =
        settings.servo.statePulseUs[otherState - SERVO_STATE_MIN];
    const bool stateBefore = state < otherState;
    const bool ordered =
        settings.servo.invert
            ? (stateBefore ? pulseUs > otherPulse : pulseUs < otherPulse)
            : (stateBefore ? pulseUs < otherPulse : pulseUs > otherPulse);
    if (!ordered) {
      return false;
    }
  }
  return true;
}

bool servoPulseForReferencedState(uint8_t state, uint16_t &pulseUs) {
  if (!servoRuntime.stateInitialized ||
      servoRuntime.referenceState < SERVO_STATE_MIN ||
      servoRuntime.referenceState > SERVO_STATE_MAX ||
      state < SERVO_STATE_MIN ||
      state > SERVO_STATE_MAX) {
    return false;
  }
  if (!servoStatePulseCalibrated(state)) {
    return false;
  }
  pulseUs = settings.servo.statePulseUs[state - SERVO_STATE_MIN];
  return pulseUs >= SERVO_MIN_US && pulseUs <= SERVO_MAX_US;
}

bool servoTube1ReferenceBounds(uint16_t &minimumPulseUs,
                               uint16_t &maximumPulseUs) {
  bool found = false;
  minimumPulseUs = SERVO_MAX_US;
  maximumPulseUs = SERVO_MIN_US;
  for (uint16_t candidate = SERVO_MIN_US;
       candidate <= SERVO_MAX_US;
       ++candidate) {
    bool allTargetsSafe = true;
    for (uint8_t state = SERVO_STATE_MIN;
         state <= SERVO_STATE_MAX;
         ++state) {
      uint16_t ignoredPulseUs = 0;
      if (!servoPulseForAnchor(candidate,
                               SERVO_STATE_MIN,
                               state,
                               ignoredPulseUs)) {
        allTargetsSafe = false;
        break;
      }
    }
    if (allTargetsSafe) {
      if (!found) {
        minimumPulseUs = candidate;
        found = true;
      }
      maximumPulseUs = candidate;
    }
  }
  return found;
}

bool ensureServoTimerStarted(
    uint16_t pulseUs,
    ServoPulseSource source) {
  if (!servoPwmWriteMicroseconds(pulseUs, source)) {
    servoRuntime.outputEnabled = false;
    servoRuntime.error = ServoError::TimerConfiguration;
    return false;
  }
  servoRuntime.outputEnabled = true;
  servoRuntime.currentPulseUs = pulseUs;
  servoRuntime.pulseReferenceKnown = true;
  return true;
}

void disableServoOutput() {
  servoPwmStop();
  servoRuntime.outputEnabled = false;
}

void stopServoForSafety(ServoSafetyReason reason) {
  disableServoOutput();
  servoRuntime.motion = ServoMotion::Idle;
  servoRuntime.targetPulseUs = servoRuntime.currentPulseUs;
  servoRuntime.rampPulseUs = servoRuntime.currentPulseUs;
  servoRuntime.stateInitialized = false;
  servoRuntime.currentState = 0;
  servoRuntime.targetState = 0;
  servoRuntime.referenceState = 0;
  servoRuntime.referencePulseUs = servoRuntime.currentPulseUs;
  servoRuntime.pulseReferenceKnown = false;
  servoRuntime.referencePulseAssumed = false;
  servoRuntime.safetyReason = reason;
  servoRuntime.error = ServoError::None;
}

uint8_t servoActiveSpeedDegPerSec() {
  if (servoRuntime.motion != ServoMotion::Idle) {
    return settings.servo.speedDegPerSec;
  }
  return 0;
}

bool serviceServoRamp(uint32_t now, ServoPulseSource source) {
  const uint32_t elapsedMs = now - servoRuntime.lastRampMs;
  if (elapsedMs < SERVO_RAMP_INTERVAL_MS) {
    return servoRuntime.currentPulseUs == servoRuntime.targetPulseUs;
  }
  servoRuntime.lastRampMs = now;

  // Limit a delayed loop iteration to 100 ms of ramp progress. A UART/I2C
  // stall must not turn into one large, high-current servo jump.
  const uint32_t boundedElapsedMs = elapsedMs > 100U ? 100U : elapsedMs;
  const uint8_t configuredSpeed =
      settings.servo.speedDegPerSec >= MIN_SERVO_SPEED_DPS &&
              settings.servo.speedDegPerSec <= MAX_SERVO_SPEED_DPS
          ? settings.servo.speedDegPerSec
          : DEFAULT_SERVO_SPEED_DPS;
  const float stepAngleDeg =
      configuredSpeed *
      static_cast<float>(boundedElapsedMs) / 1000.0f;
  const float currentAngleDeg =
      servoAngleForPulse(servoRuntime.rampPulseUs);
  const float targetAngleDeg =
      servoAngleForPulse(static_cast<float>(servoRuntime.targetPulseUs));
  const float angleDistanceDeg = targetAngleDeg - currentAngleDeg;
  if (fabsf(angleDistanceDeg) <= stepAngleDeg) {
    servoRuntime.rampPulseUs =
        static_cast<float>(servoRuntime.targetPulseUs);
  } else {
    servoRuntime.rampPulseUs = servoPulseForAngle(
        currentAngleDeg +
        (angleDistanceDeg > 0.0f ? stepAngleDeg : -stepAngleDeg));
  }

  long nextPulse = lroundf(servoRuntime.rampPulseUs);
  if (nextPulse < SERVO_MIN_US) {
    nextPulse = SERVO_MIN_US;
  } else if (nextPulse > SERVO_MAX_US) {
    nextPulse = SERVO_MAX_US;
  }
  if (static_cast<uint16_t>(nextPulse) != servoRuntime.currentPulseUs) {
    if (!ensureServoTimerStarted(static_cast<uint16_t>(nextPulse), source)) {
      servoRuntime.rampPulseUs = servoRuntime.currentPulseUs;
      servoRuntime.motion = ServoMotion::Idle;
      return false;
    }
  }
  return servoRuntime.rampPulseUs ==
         static_cast<float>(servoRuntime.targetPulseUs);
}

bool startServoStateMove(uint8_t targetState) {
  if (!servoRuntime.stateInitialized ||
      servoRuntime.referencePulseAssumed ||
      targetState < SERVO_STATE_MIN ||
      targetState > SERVO_STATE_MAX ||
      servoRuntime.motion != ServoMotion::Idle) {
    return false;
  }
  if (!servoStatePulseCalibrated(targetState)) {
    servoRuntime.error = ServoError::TargetNotCalibrated;
    servoRuntime.safetyReason = ServoSafetyReason::Ready;
    return false;
  }
  uint16_t targetPulse = servoRuntime.currentPulseUs;
  if (!servoPulseForReferencedState(targetState, targetPulse)) {
    servoRuntime.safetyReason = ServoSafetyReason::PulseRange;
    servoRuntime.error = ServoError::TargetPulseRange;
    return false;
  }

  servoRuntime.targetState = targetState;
  servoRuntime.targetPulseUs = targetPulse;
  servoRuntime.safetyReason = ServoSafetyReason::Ready;
  servoRuntime.error = ServoError::None;

  // Start from the pulse currently being generated and let serviceServo()
  // advance toward the calibrated target once per 20 ms frame. This keeps
  // UART/I2C/motor services non-blocking while making servospeed effective.
  if (!ensureServoTimerStarted(
          servoRuntime.currentPulseUs, ServoPulseSource::TubeTarget)) {
    return false;
  }
  servoRuntime.rampPulseUs = servoRuntime.currentPulseUs;
  if (servoRuntime.currentPulseUs == targetPulse) {
    servoRuntime.currentState = targetState;
    servoRuntime.motion = ServoMotion::Idle;
    return true;
  }

  const uint32_t now = millis();
  servoRuntime.motion = ServoMotion::StateMove;
  servoRuntime.motionStartedMs = now;
  servoRuntime.lastRampMs = now;
  return true;
}

bool startServoFixedMove(uint16_t targetPulseUs) {
  if (targetPulseUs < SERVO_MIN_US ||
      targetPulseUs > SERVO_MAX_US ||
      servoRuntime.motion != ServoMotion::Idle) {
    return false;
  }

  // Fixed-pulse and one-click step controls deliberately invalidate the
  // logical state: the final pulse may not equal any saved state target.
  servoRuntime.stateInitialized = false;
  servoRuntime.currentState = 0;
  servoRuntime.targetState = 0;
  servoRuntime.referenceState = 0;
  servoRuntime.referencePulseAssumed = false;
  servoRuntime.targetPulseUs = targetPulseUs;
  servoRuntime.rampPulseUs = servoRuntime.currentPulseUs;
  servoRuntime.safetyReason = ServoSafetyReason::Ready;
  servoRuntime.error = ServoError::None;

  // Re-enable PWM at the last commanded pulse, then advance gradually. The
  // first command after a power cycle starts from the deterministic 1500 us
  // software baseline because a three-wire servo has no position feedback.
  if (!ensureServoTimerStarted(
          servoRuntime.currentPulseUs, ServoPulseSource::FixedTest)) {
    return false;
  }
  if (servoRuntime.currentPulseUs == targetPulseUs) {
    servoRuntime.motion = ServoMotion::Idle;
    return true;
  }

  const uint32_t now = millis();
  servoRuntime.motion = ServoMotion::FixedMove;
  servoRuntime.motionStartedMs = now;
  servoRuntime.lastRampMs = now;
  return true;
}

bool startServoWiggle() {
  if (!servoRuntime.stateInitialized ||
      servoRuntime.referencePulseAssumed ||
      servoRuntime.motion != ServoMotion::Idle) {
    return false;
  }
  const uint32_t now = millis();
  servoRuntime.homePulseUs = servoRuntime.currentPulseUs;
  servoRuntime.targetPulseUs = servoRuntime.homePulseUs;
  servoRuntime.rampPulseUs = servoRuntime.currentPulseUs;
  if (!ensureServoTimerStarted(
          servoRuntime.homePulseUs,
          ServoPulseSource::Wiggle)) {
    return false;
  }
  servoRuntime.motion = ServoMotion::Wiggle;
  servoRuntime.motionStartedMs = now;
  servoRuntime.lastRampMs = now;
  servoRuntime.nextStepMs = now;
  servoRuntime.wiggleDirection =
      servoRuntime.currentState == SERVO_STATE_MAX ? -1 : 1;
  servoRuntime.safetyReason = ServoSafetyReason::Ready;
  return true;
}

void serviceServo() {
  const uint32_t now = millis();
  if (servoRuntime.motion == ServoMotion::Idle) {
    return;
  }

  if (servoRuntime.motion == ServoMotion::StateMove) {
    if (serviceServoRamp(now, ServoPulseSource::TubeTarget) &&
        servoRuntime.outputEnabled) {
      servoRuntime.currentState = servoRuntime.targetState;
      servoRuntime.motion = ServoMotion::Idle;
    }
    return;
  }

  if (servoRuntime.motion == ServoMotion::FixedMove) {
    if (serviceServoRamp(now, ServoPulseSource::FixedTest) &&
        servoRuntime.outputEnabled) {
      servoRuntime.motion = ServoMotion::Idle;
    }
    return;
  }

  if (now - servoRuntime.motionStartedMs >= SERVO_WIGGLE_TOTAL_MS) {
    servoRuntime.targetPulseUs = servoRuntime.homePulseUs;
    if (serviceServoRamp(now, ServoPulseSource::Wiggle)) {
      servoRuntime.motion = ServoMotion::Idle;
    }
    return;
  }

  if (static_cast<int32_t>(now - servoRuntime.nextStepMs) >= 0) {
    const int physicalDirection =
        settings.servo.invert ? -servoRuntime.wiggleDirection
                              : servoRuntime.wiggleDirection;
    float targetAngleDeg =
        servoAngleForPulse(static_cast<float>(servoRuntime.homePulseUs)) +
        physicalDirection * settings.servo.wiggleAngleDeg;
    if (targetAngleDeg < -SERVO_MAX_ABSOLUTE_ANGLE_DEG) {
      targetAngleDeg = -SERVO_MAX_ABSOLUTE_ANGLE_DEG;
    } else if (targetAngleDeg > SERVO_MAX_ABSOLUTE_ANGLE_DEG) {
      targetAngleDeg = SERVO_MAX_ABSOLUTE_ANGLE_DEG;
    }
    servoRuntime.targetPulseUs =
        static_cast<uint16_t>(lroundf(servoPulseForAngle(targetAngleDeg)));

    if (servoRuntime.currentState == SERVO_STATE_MIN) {
      servoRuntime.wiggleDirection =
          servoRuntime.wiggleDirection > 0 ? 0 : 1;
    } else if (servoRuntime.currentState == SERVO_STATE_MAX) {
      servoRuntime.wiggleDirection =
          servoRuntime.wiggleDirection < 0 ? 0 : -1;
    } else {
      servoRuntime.wiggleDirection =
          servoRuntime.wiggleDirection > 0 ? -1 : 1;
    }
    servoRuntime.nextStepMs = now + SERVO_WIGGLE_STEP_MS;
  }
  serviceServoRamp(now, ServoPulseSource::Wiggle);
}

Direction applyInvert(Direction direction, uint8_t invert);
void brakeMotor(uint8_t motorIndex, uint16_t durationMs);
BrakePointOutputAction filterBrakePointMotorOutput(uint8_t axisIndex,
                                                   Direction physicalDirection,
                                                   uint8_t requestedPwm);
bool brakePointHasExclusiveControl(uint8_t axisIndex);
void resetBrakePointRuntime(uint8_t axisIndex);
void noteBrakePointNeutral(uint8_t axisIndex);
void updateBrakePointSensorState(uint8_t axisIndex,
                                 float previousAngleDeg,
                                 float currentAngleDeg,
                                 bool hadPreviousAngle);

void writeMotorPwm(uint8_t pin, uint8_t pwm) {
  if (pwm == 0) {
    forceMotorPinLow(pin);
  } else {
    analogWrite(pin, pwm);
  }
}

void configureMotorPins() {
  analogWriteResolution(8);
  analogWriteFrequency(20000);

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    forceMotorPinLow(motorPins[i].rpwm);
    forceMotorPinLow(motorPins[i].lpwm);
    forceMotorPinLow(motorPins[i].en);
    digitalWrite(motorPins[i].en, LOW);
  }
}

void driveMotorHardware(uint8_t motorIndex, Direction direction, uint8_t pwm) {
  if (motorIndex >= MOTOR_COUNT) {
    return;
  }

  motors[motorIndex].direction = direction;
  motors[motorIndex].pwm = pwm;
  motors[motorIndex].braking = false;

  if (direction == Direction::Stop || pwm == 0) {
    digitalWrite(motorPins[motorIndex].en, LOW);
    writeMotorPwm(motorPins[motorIndex].rpwm, 0);
    writeMotorPwm(motorPins[motorIndex].lpwm, 0);
    motors[motorIndex].direction = Direction::Stop;
    motors[motorIndex].pwm = 0;
    return;
  }

  if (direction == Direction::Forward) {
    writeMotorPwm(motorPins[motorIndex].lpwm, 0);
    writeMotorPwm(motorPins[motorIndex].rpwm, pwm);
  } else {
    writeMotorPwm(motorPins[motorIndex].rpwm, 0);
    writeMotorPwm(motorPins[motorIndex].lpwm, pwm);
  }
  digitalWrite(motorPins[motorIndex].en, HIGH);
}

void driveMotorRaw(uint8_t motorIndex, Direction direction, uint8_t pwm) {
  if (motorIndex < 3 && direction != Direction::Stop && pwm > 0) {
    const BrakePointOutputAction action =
        filterBrakePointMotorOutput(motorIndex, direction, pwm);
    if (action == BrakePointOutputAction::Ignore) {
      return;
    }
    if (action == BrakePointOutputAction::Brake) {
      if (!motors[motorIndex].braking) {
        brakeMotor(motorIndex, BRAKE_PULSE_MS);
      }
      return;
    }
  }
  driveMotorHardware(motorIndex, direction, pwm);
}

void coastMotor(uint8_t motorIndex) {
  driveMotorRaw(motorIndex, Direction::Stop, 0);
}

void brakeMotor(uint8_t motorIndex, uint16_t durationMs) {
  if (motorIndex >= MOTOR_COUNT) {
    return;
  }

  motors[motorIndex].direction = Direction::Stop;
  motors[motorIndex].pwm = 0;
  motors[motorIndex].braking = true;
  motors[motorIndex].brakeUntilMs = millis() + durationMs;

  analogWrite(motorPins[motorIndex].rpwm, MAX_PWM);
  analogWrite(motorPins[motorIndex].lpwm, MAX_PWM);
  digitalWrite(motorPins[motorIndex].en, HIGH);
}

void clearKeybindRuntime() {
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    keybindForwardActive[i] = false;
    keybindBackwardActive[i] = false;
  }
}

void clearAxisKeybindRuntime(uint8_t axisIndex) {
  if (axisIndex >= AXIS_COUNT) {
    return;
  }
  keybindForwardActive[axisIndex] = false;
  keybindBackwardActive[axisIndex] = false;
}

bool anyMotorOutputActive() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (motors[i].direction != Direction::Stop || motors[i].pwm != 0 || motors[i].braking) {
      return true;
    }
    if (digitalRead(motorPins[i].en) == HIGH) {
      return true;
    }
  }
  return false;
}

void resetPositionControllerRuntime(uint8_t axisIndex) {
  if (axisIndex >= 3) {
    return;
  }
  AxisRuntime &runtime = axisRuntime[axisIndex];
  runtime.pidOutputDirection = Direction::Stop;
  runtime.pidOutputPwm = 0;
  runtime.pidLastMs = millis();
  runtime.pidReverseHoldUntilMs = 0;
  runtime.holdSettled = false;
}

void resetBrakePointRuntime(uint8_t axisIndex) {
  if (axisIndex >= 3) {
    return;
  }
  AxisRuntime &runtime = axisRuntime[axisIndex];
  runtime.activeBrakePointBoundary = BP_INVALID_BOUNDARY;
  runtime.brakePointApproach = BrakePointApproach::Inactive;
  runtime.brakePointPhase = BrakePointPhase::Inactive;
  runtime.brakePointFaultReason = BrakePointFaultReason::None;
  runtime.brakePointLeadAtTriggerDeg = 0.0f;
  runtime.brakePointRecoveryTargetDeg = 0.0f;
  runtime.brakePointRecoveryDirection = Direction::Stop;
  runtime.brakePointPhaseStartedMs = 0;
  runtime.brakePointSettledSamples = 0;
  runtime.brakePointNeutralSeen = false;
  runtime.brakePointRetreatCommandSeen = false;
  runtime.brakePointRecoveryOutputAuthorized = false;
  runtime.brakePointRecoveryLastDistanceDeg = 0.0f;
  runtime.brakePointRecoveryLastProgressMs = 0;
  brakePointFaultBits &= static_cast<uint8_t>(~(1U << axisIndex));
}

bool brakePointHasExclusiveControl(uint8_t axisIndex) {
  if (axisIndex >= 3) {
    return false;
  }
  const BrakePointPhase phase = axisRuntime[axisIndex].brakePointPhase;
  return phase == BrakePointPhase::Braking ||
         phase == BrakePointPhase::Settling ||
         phase == BrakePointPhase::Recovering ||
         phase == BrakePointPhase::Fault;
}

bool anyBrakePointHasExclusiveControl() {
  for (uint8_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
    if (brakePointHasExclusiveControl(axisIndex)) {
      return true;
    }
  }
  return false;
}

void noteBrakePointNeutral(uint8_t axisIndex) {
  if (axisIndex < 3 &&
      axisRuntime[axisIndex].brakePointPhase != BrakePointPhase::Inactive) {
    axisRuntime[axisIndex].brakePointNeutralSeen = true;
  }
}

void coastPositionControllerAxis(uint8_t axisIndex) {
  if (axisIndex >= 3) {
    return;
  }
  coastMotor(axisIndex);
  axisRuntime[axisIndex].pidOutputDirection = Direction::Stop;
  axisRuntime[axisIndex].pidOutputPwm = 0;
  axisRuntime[axisIndex].pidLastMs = millis();
}

uint8_t rampPwmToward(uint8_t current, uint8_t target, uint32_t dtMs) {
  if (current == target) {
    return target;
  }
  if (dtMs > 100) {
    dtMs = 100;
  }
  uint16_t maxDelta = static_cast<uint16_t>((static_cast<uint32_t>(POSITION_OUTPUT_RAMP_PWM_PER_SEC) * dtMs) / 1000UL);
  if (maxDelta < 1) {
    maxDelta = 1;
  }
  if (target > current) {
    const uint16_t next = static_cast<uint16_t>(current) + maxDelta;
    return next > target ? target : static_cast<uint8_t>(next);
  }
  return current > maxDelta + target ? static_cast<uint8_t>(current - maxDelta) : target;
}

void cancelTargets() {
  for (uint8_t i = 0; i < 3; ++i) {
    targets[i].active = false;
    targets[i].hold = false;
    resetPositionControllerRuntime(i);
  }
}

bool getFreshAxisAngle(uint8_t axisIndex, float &angleDeg);
void AS5600_UpdateFast();
void MotionControl_UpdateFast();
void Telemetry_Update();
void stopAll(bool useConfiguredStopMode);

int8_t configuredAS5600Channel(uint8_t axisIndex) {
  if (axisIndex >= AXIS_COUNT) {
    return -1;
  }
  const uint8_t channel = settings.axis[axisIndex].as5600Channel;
  if (channel > 7) {
    return -1;
  }
  return static_cast<int8_t>(channel);
}

class SensorManager {
 public:
  void begin() {
    beginWire();
    consecutiveErrors_ = 0;
    recoveryPending_ = false;
    recoveryBackoffMs_ = I2C_RECOVERY_INITIAL_BACKOFF_MS;
    nextRecoveryAllowedMs_ = millis() + recoveryBackoffMs_;
  }

  bool readAngle(uint8_t axisIndex, float &angleDeg) {
    uint16_t rawAngle = 0;
    return readAngle(axisIndex, rawAngle, angleDeg);
  }

  bool readAngle(uint8_t axisIndex, uint16_t &rawAngle, float &angleDeg) {
    if (axisIndex > 2) {
      return false;
    }
    const int8_t channel = configuredAS5600Channel(axisIndex);
    if (channel < 0) {
      return false;
    }
    return readAS5600Angle(static_cast<uint8_t>(channel), rawAngle, angleDeg);
  }

  bool selectMuxChannel(uint8_t channel) {
    if (channel > 7) {
      return false;
    }
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(static_cast<uint8_t>(1U << channel));
    return Wire.endTransmission() == 0;
  }

  bool readAS5600Angle(uint8_t channel, uint16_t &rawAngle, float &angleDeg) {
    if (!selectMuxChannel(channel)) {
      recordFailure();
      return false;
    }
    uint8_t highByte = 0;
    uint8_t lowByte = 0;
    if (!readRegister16(AS5600_ADDR, AS5600_RAW_ANGLE_REG, highByte, lowByte)) {
      recordFailure();
      return false;
    }
    rawAngle = static_cast<uint16_t>(((highByte & 0x0F) << 8) | lowByte);
    angleDeg = static_cast<float>(rawAngle) * AS5600_DEGREES_PER_COUNT;
    recordSuccess();
    return true;
  }

  bool readAS5600Status(uint8_t channel, uint8_t &status) {
    if (!selectMuxChannel(channel)) {
      recordFailure();
      return false;
    }
    if (!readRegister8(AS5600_ADDR, AS5600_STATUS_REG, status)) {
      recordFailure();
      return false;
    }
    recordSuccess();
    return true;
  }

  bool serviceRecovery() {
    if (!recoveryPending_) {
      return false;
    }

    const uint32_t nowMs = millis();
    if (static_cast<int32_t>(nowMs - nextRecoveryAllowedMs_) < 0) {
      return false;
    }

    // PB8/PB9 are dedicated to I2C in this project. Wire.end() resets only
    // the selected I2C peripheral; Wire.begin() then runs the STM32 core's
    // bounded SDA/SCL recovery before reinitializing it.
    Wire.end();
    beginWire();

    ++recoveryCount_;
    consecutiveErrors_ = 0;
    recoveryPending_ = false;
    nextRecoveryAllowedMs_ = nowMs + recoveryBackoffMs_;
    recoveryBackoffMs_ = nextI2cRecoveryBackoffMs(recoveryBackoffMs_);
    return true;
  }

  uint32_t totalErrorCount() const {
    return totalErrorCount_;
  }

  uint32_t recoveryCount() const {
    return recoveryCount_;
  }

 private:
  void beginWire() {
    Wire.setSDA(TCA9548A_SDA_PIN);
    Wire.setSCL(TCA9548A_SCL_PIN);
    Wire.begin();
    Wire.setClock(AS5600_I2C_CLOCK_HZ);
  }

  void recordFailure() {
    if (consecutiveErrors_ < UINT16_MAX) {
      ++consecutiveErrors_;
    }
    ++totalErrorCount_;
    if (consecutiveErrors_ >= I2C_RECOVERY_ERROR_THRESHOLD) {
      recoveryPending_ = true;
    }
  }

  void recordSuccess() {
    consecutiveErrors_ = 0;
    recoveryPending_ = false;
    recoveryBackoffMs_ = I2C_RECOVERY_INITIAL_BACKOFF_MS;
    nextRecoveryAllowedMs_ = millis() + recoveryBackoffMs_;
  }

  bool readRegister8(uint8_t address, uint8_t reg, uint8_t &value) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
      return false;
    }
    const uint8_t received = Wire.requestFrom(address, static_cast<uint8_t>(1));
    if (received != 1 || Wire.available() < 1) {
      return false;
    }
    value = Wire.read();
    return true;
  }

  bool readRegister16(uint8_t address, uint8_t reg, uint8_t &highByte, uint8_t &lowByte) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
      return false;
    }
    const uint8_t received = Wire.requestFrom(address, static_cast<uint8_t>(2));
    if (received != 2 || Wire.available() < 2) {
      return false;
    }
    highByte = Wire.read();
    lowByte = Wire.read();
    return true;
  }

  uint16_t consecutiveErrors_ = 0;
  uint32_t totalErrorCount_ = 0;
  uint32_t recoveryCount_ = 0;
  uint32_t recoveryBackoffMs_ = I2C_RECOVERY_INITIAL_BACKOFF_MS;
  uint32_t nextRecoveryAllowedMs_ = 0;
  bool recoveryPending_ = false;
};

SensorManager sensors;

uint8_t scaledManualPwm(uint8_t axisIndex, int requestedPwm, bool positiveCommand) {
  const AxisConfig &cfg = settings.axis[axisIndex];
  const int base = requestedPwm < 0 ? cfg.defaultPwm : requestedPwm;
  const uint16_t gain = positiveCommand ? cfg.posGain : cfg.negGain;
  const int scaled = static_cast<int>((static_cast<int32_t>(base) * gain) / 1000L);
  return clampPwm(scaled, cfg.maxPwm);
}

Direction applyInvert(Direction direction, uint8_t invert) {
  if (!invert || direction == Direction::Stop) {
    return direction;
  }
  return direction == Direction::Forward ? Direction::Backward : Direction::Forward;
}

constexpr int16_t signedPwm(Direction direction, uint8_t pwm) {
  if (direction == Direction::Stop || pwm == 0) {
    return 0;
  }
  return direction == Direction::Backward
             ? -static_cast<int16_t>(pwm)
             : static_cast<int16_t>(pwm);
}

static_assert(signedPwm(Direction::Backward, 191) == -191,
              "sendpwm must preserve backward direction");

void resetElevatorPwmRuntime() {
  elevatorPwmRuntime.requested[0] = 0;
  elevatorPwmRuntime.requested[1] = 0;
  elevatorPwmRuntime.actual[0] = 0;
  elevatorPwmRuntime.actual[1] = 0;
}

void drivePhysicalAxis(uint8_t axisIndex, Direction direction, int requestedPwm, bool cancelTarget = true) {
  if (axisIndex < 3 && cancelTarget && brakePointHasExclusiveControl(axisIndex)) {
    return;
  }
  AxisConfig &cfg = settings.axis[axisIndex];
  const bool positiveCommand = direction == Direction::Forward;
  const uint8_t pwm = scaledManualPwm(axisIndex, requestedPwm, positiveCommand);
  direction = applyInvert(direction, cfg.invert);
  driveMotorRaw(axisIndex, direction, pwm);
  if (cancelTarget && axisIndex < 3) {
    targets[axisIndex].active = false;
    targets[axisIndex].hold = false;
    resetPositionControllerRuntime(axisIndex);
  }
}

void driveWristVirtual(uint8_t virtualAxisIndex, Direction direction, int requestedPwm) {
  AxisConfig &cfg = settings.axis[virtualAxisIndex];
  const bool positiveCommand = direction == Direction::Forward;
  const uint8_t pwm = scaledManualPwm(virtualAxisIndex, requestedPwm, positiveCommand);
  direction = applyInvert(direction, cfg.invert);

  if (virtualAxisIndex == 3) {
    // Axis 4 is pitch: both wrist motors rotate in the same direction.
    driveMotorRaw(3, direction, pwm);
    driveMotorRaw(4, direction, pwm);
  } else {
    // Axis 5 is twist: wrist motors rotate in opposite directions.
    driveMotorRaw(3, direction, pwm);
    driveMotorRaw(4, direction == Direction::Forward ? Direction::Backward : Direction::Forward, pwm);
  }
}

bool driveArmAxisSilent(uint8_t oneBasedAxis, Direction direction, int requestedPwm) {
  if (oneBasedAxis < 1 || oneBasedAxis > AXIS_COUNT) {
    return false;
  }
  const uint8_t axisIndex = oneBasedAxis - 1;
  if (mode != SystemMode::Arm && mode != SystemMode::ArmTest) {
    return false;
  }
  if (mode == SystemMode::Arm && axisIndex < 3 &&
      anyBrakePointHasExclusiveControl()) {
    return false;
  }

  if (axisIndex <= 2) {
    drivePhysicalAxis(axisIndex, direction, requestedPwm);
  } else if (axisIndex == 3 || axisIndex == 4) {
    driveWristVirtual(axisIndex, direction, requestedPwm);
  } else {
    drivePhysicalAxis(5, direction, requestedPwm);
  }
  return true;
}

void driveArmAxis(uint8_t oneBasedAxis, Direction direction, int requestedPwm) {
  if (oneBasedAxis < 1 || oneBasedAxis > AXIS_COUNT) {
    sendErr("BAD_AXIS");
    return;
  }
  if (mode != SystemMode::Arm && mode != SystemMode::ArmTest) {
    sendErr("MODE_NOT_ARM_OR_ARMTEST");
    return;
  }
  if (!driveArmAxisSilent(oneBasedAxis, direction, requestedPwm)) {
    sendErr("DRIVE_FAILED");
    return;
  }
  sendOk("DRIVE");
}

void stopAxis(uint8_t oneBasedAxis) {
  if (oneBasedAxis < 1 || oneBasedAxis > AXIS_COUNT) {
    sendErr("BAD_AXIS");
    return;
  }

  const uint8_t axisIndex = oneBasedAxis - 1;
  if (mode == SystemMode::Drill &&
      (axisIndex == 3 || axisIndex == 4)) {
    resetElevatorPwmRuntime();
  }
  if (axisIndex < 3) {
    targets[axisIndex].active = false;
    targets[axisIndex].hold = false;
    resetPositionControllerRuntime(axisIndex);
  }

  StopMode stopMode = static_cast<StopMode>(settings.axis[axisIndex].stopMode);
  if (axisIndex >= 3 && stopMode == StopMode::Hold) {
    stopMode = StopMode::Coast;
  }
  if (mode == SystemMode::ArmTest) {
    if (stopMode == StopMode::Hold) {
      stopMode = StopMode::Coast;
    } else if (stopMode == StopMode::Hybrid) {
      stopMode = StopMode::Brake;
    }
  }
  if (axisIndex < 3 && mode == SystemMode::Arm &&
      (stopMode == StopMode::Hold || stopMode == StopMode::Hybrid)) {
    float holdAngle = 0.0f;
    if (getFreshAxisAngle(axisIndex, holdAngle)) {
      targets[axisIndex].targetDeg = holdAngle;
      targets[axisIndex].active = true;
      targets[axisIndex].hold = true;
    } else if (stopMode == StopMode::Hold) {
      stopMode = StopMode::Coast;
    }
  }

  auto stopMotorByMode = [&](uint8_t motorIndex, StopMode selected) {
    if (selected == StopMode::Brake || selected == StopMode::Hybrid) {
      brakeMotor(motorIndex, BRAKE_PULSE_MS);
    } else {
      coastMotor(motorIndex);
    }
  };

  if (axisIndex == 3 || axisIndex == 4) {
    stopMotorByMode(3, stopMode);
    stopMotorByMode(4, stopMode);
  } else {
    stopMotorByMode(axisIndex, stopMode);
  }
}

void stopAll(bool useConfiguredStopMode) {
  cancelTargets();
  clearKeybindRuntime();
  resetElevatorPwmRuntime();
  stopServoForSafety(ServoSafetyReason::Stop);
  for (uint8_t i = 0; i < DRILL_LED_COUNT; ++i) {
    setDrillLed(i, false);
  }
  for (uint8_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
    resetBrakePointRuntime(axisIndex);
  }
  if (useConfiguredStopMode &&
      (mode == SystemMode::Arm || mode == SystemMode::ArmTest)) {
    for (uint8_t axis = 1; axis <= AXIS_COUNT; ++axis) {
      stopAxis(axis);
    }
  } else {
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
      coastMotor(i);
    }
  }
}

void enterMode(SystemMode newMode) {
  stopAll(false);
  if (newMode != SystemMode::Drill) {
    servoRuntime.safetyReason = ServoSafetyReason::SafeMode;
  }
  mode = newMode;

  LinkSerial.print("OK MODE ");
  LinkSerial.println(modeName(mode));
}

void driveDrillMotor(uint8_t motorIndex, Direction direction, int requestedPwm, const DrillConfig &cfg) {
  const int base = requestedPwm < 0 ? cfg.defaultPwm : requestedPwm;
  uint8_t pwm = clampPwm(base, cfg.maxPwm);
  direction = applyInvert(direction, cfg.invert);
  driveMotorRaw(motorIndex, direction, pwm);
}

void driveElevatorMotor(uint8_t motorIndex,
                        Direction direction,
                        int requestedPwm) {
  if (motorIndex < 3 || motorIndex > 4) {
    return;
  }
  const int base = requestedPwm < 0
                       ? settings.elevator.defaultPwm
                       : requestedPwm;
  const uint8_t pwm = clampPwm(base, settings.elevator.maxPwm);
  direction = applyInvert(direction, settings.elevator.invert);

  const uint8_t elevatorMotorIndex = motorIndex - 3;
  const uint8_t actualPwm = scalePwmByPercent(
      pwm, settings.elevator.sendPwmPercent[elevatorMotorIndex]);
  elevatorPwmRuntime.requested[elevatorMotorIndex] =
      signedPwm(direction, pwm);
  elevatorPwmRuntime.actual[elevatorMotorIndex] =
      signedPwm(direction, actualPwm);
  driveMotorRaw(motorIndex, direction, actualPwm);
}

void stopDrillGroup(bool elevator) {
  if (elevator) {
    resetElevatorPwmRuntime();
    coastMotor(3);
    coastMotor(4);
  } else {
    coastMotor(5);
  }
}

bool parseIntToken(const char *token, int &value) {
  if (token == nullptr || *token == '\0') {
    return false;
  }
  char *end = nullptr;
  long parsed = strtol(token, &end, 10);
  if (*end != '\0') {
    return false;
  }
  value = static_cast<int>(parsed);
  return true;
}

bool parseFloatToken(const char *token, float &value) {
  if (token == nullptr || *token == '\0') {
    return false;
  }
  char *end = nullptr;
  float parsed = strtof(token, &end);
  if (*end != '\0' || !isfinite(parsed)) {
    return false;
  }
  value = parsed;
  return true;
}

bool parseAxis(const char *token, uint8_t &axisOneBased) {
  int value = 0;
  if (!parseIntToken(token, value) || value < 1 || value > 6) {
    return false;
  }
  axisOneBased = static_cast<uint8_t>(value);
  return true;
}

bool parseBts(const char *token, uint8_t &btsOneBased) {
  if (token == nullptr || strncmp(token, "bts", 3) != 0) {
    return false;
  }
  int value = 0;
  if (!parseIntToken(token + 3, value) || value < 1 || value > 6) {
    return false;
  }
  btsOneBased = static_cast<uint8_t>(value);
  return true;
}

bool parseSendPwmPercent(const char *token, uint8_t &percent) {
  float value = 0.0f;
  if (!parseFloatToken(token, value) || value < 0.0f || value > 1.0f) {
    return false;
  }
  const float scaled = value * SENDPWM_PERCENT_MAX;
  const int rounded = static_cast<int>(lroundf(scaled));
  if (fabsf(scaled - rounded) > 0.001f ||
      rounded < 0 || rounded > SENDPWM_PERCENT_MAX) {
    return false;
  }
  percent = static_cast<uint8_t>(rounded);
  return true;
}

bool inRange(int value, int minValue, int maxValue) {
  return value >= minValue && value <= maxValue;
}

const char *brakePointModeName(BrakePointMode value) {
  switch (value) {
    case BrakePointMode::Wall: return "WALL";
    case BrakePointMode::BlockIncrease: return "BLOCK_INCREASE";
    case BrakePointMode::BlockDecrease: return "BLOCK_DECREASE";
    default: return "OFF";
  }
}

void printBrakePointBoundaryId(uint8_t boundaryIndex) {
  if (boundaryIndex < BP_SLOT_COUNT) {
    LinkSerial.print(boundaryIndex + 1);
  } else {
    LinkSerial.print("NONE");
  }
}

bool parseBrakePointMode(const char *text, BrakePointMode &modeValue) {
  if (strcmp(text, "wall") == 0) {
    modeValue = BrakePointMode::Wall;
    return true;
  }
  if (strcmp(text, "blockincrease") == 0 ||
      strcmp(text, "block_increase") == 0 ||
      strcmp(text, "blockinc") == 0) {
    modeValue = BrakePointMode::BlockIncrease;
    return true;
  }
  if (strcmp(text, "blockdecrease") == 0 ||
      strcmp(text, "block_decrease") == 0 ||
      strcmp(text, "blockdec") == 0) {
    modeValue = BrakePointMode::BlockDecrease;
    return true;
  }
  if (strcmp(text, "off") == 0 || strcmp(text, "disabled") == 0) {
    modeValue = BrakePointMode::Disabled;
    return true;
  }
  return false;
}

struct BrakePointBoundaryView {
  BrakePointMode mode = BrakePointMode::Disabled;
  float angleDeg = 0.0f;
  uint8_t boundaryIndex = BP_INVALID_BOUNDARY;
};

bool brakePointBoundaryFromConfig(
    const BrakePointSlot slots[BP_SLOT_COUNT],
    uint8_t boundaryIndex,
    BrakePointBoundaryView &boundary) {
  if (boundaryIndex >= BP_SLOT_COUNT) {
    return false;
  }
  boundary.boundaryIndex = boundaryIndex;
  boundary.mode =
      static_cast<BrakePointMode>(slots[boundaryIndex].mode);
  boundary.angleDeg = normalize360(slots[boundaryIndex].angleDeg);
  return boundary.mode != BrakePointMode::Disabled;
}

bool getBrakePointBoundary(uint8_t axisIndex,
                           uint8_t boundaryIndex,
                           BrakePointBoundaryView &boundary) {
  if (axisIndex >= 3) {
    return false;
  }
  return brakePointBoundaryFromConfig(
      settings.brakePointSlots[axisIndex],
      boundaryIndex,
      boundary);
}

bool axisHasAnyBrakePoint(uint8_t axisIndex) {
  if (axisIndex >= 3) {
    return false;
  }
  for (uint8_t slot = 0; slot < BP_SLOT_COUNT; ++slot) {
    if (static_cast<BrakePointMode>(
            settings.brakePointSlots[axisIndex][slot].mode) !=
        BrakePointMode::Disabled) {
      return true;
    }
  }
  return false;
}

bool brakePointPathAllowed(uint8_t axisIndex,
                           float startAngleDeg,
                           float targetAngleDeg,
                           uint8_t &blockingBoundary) {
  blockingBoundary = BP_INVALID_BOUNDARY;
  if (axisIndex >= 3 || !axisHasAnyBrakePoint(axisIndex)) {
    return true;
  }
  for (uint8_t boundaryIndex = 0;
       boundaryIndex < BP_BOUNDARY_COUNT;
       ++boundaryIndex) {
    BrakePointBoundaryView boundary;
    if (!getBrakePointBoundary(axisIndex, boundaryIndex, boundary)) {
      continue;
    }
    if (bpPathCrossesForbidden(
            boundary.mode,
            startAngleDeg,
            targetAngleDeg,
            boundary.angleDeg)) {
      blockingBoundary = boundaryIndex;
      return false;
    }
  }
  return true;
}

float configuredBrakePointClearanceDeg(uint8_t axisIndex) {
  const AxisConfig &cfg = settings.axis[axisIndex];
  const BrakePointTuning &tuning = settings.brakePoint[axisIndex];
  const float tolerance = cfg.toleranceDeg > 0.0f
                              ? cfg.toleranceDeg
                              : POSITION_TOLERANCE_DEG;
  float clearance = tuning.recoveryClearanceDeg;
  if (clearance < 2.0f * tolerance) {
    clearance = 2.0f * tolerance;
  }
  if (clearance < BP_MIN_CLEARANCE_DEG) {
    clearance = BP_MIN_CLEARANCE_DEG;
  }
  const float outsideLeadMargin =
      tuning.leadMarginDeg + BP_MIN_RECOVERY_TOLERANCE_DEG;
  if (clearance < outsideLeadMargin) {
    clearance = outsideLeadMargin;
  }
  return clearance;
}

enum class BrakePointConfigError : uint8_t {
  None = 0,
  MotorActive = 1,
  TargetActive = 2,
  RuntimeActive = 3,
  BadMode = 4,
  BadAngle = 5,
  DuplicateBoundary = 6,
  RecoveryBlockedByBoundary = 7,
  BadSlot = 8
};

struct BrakePointConfigValidation {
  BrakePointConfigError error = BrakePointConfigError::None;
  uint8_t boundary = BP_INVALID_BOUNDARY;
  uint8_t otherBoundary = BP_INVALID_BOUNDARY;
};

const char *brakePointConfigErrorName(BrakePointConfigError error) {
  switch (error) {
    case BrakePointConfigError::MotorActive: return "MOTOR_ACTIVE";
    case BrakePointConfigError::TargetActive: return "TARGET_ACTIVE";
    case BrakePointConfigError::RuntimeActive: return "BP_RUNTIME_ACTIVE";
    case BrakePointConfigError::BadMode: return "BAD_MODE";
    case BrakePointConfigError::BadAngle: return "BAD_ANGLE";
    case BrakePointConfigError::DuplicateBoundary: return "DUPLICATE_BOUNDARY";
    case BrakePointConfigError::RecoveryBlockedByBoundary: return "RECOVERY_BLOCKED_BY_BP";
    case BrakePointConfigError::BadSlot: return "BAD_SLOT";
    default: return "NONE";
  }
}

const char *brakePointFaultReasonName(BrakePointFaultReason reason) {
  switch (reason) {
    case BrakePointFaultReason::RecoveryTimeout: return "RECOVERY_TIMEOUT";
    case BrakePointFaultReason::RecoveryNoProgress: return "RECOVERY_NO_PROGRESS";
    case BrakePointFaultReason::RecoveryBlocked: return "RECOVERY_BLOCKED";
    case BrakePointFaultReason::RuntimeConflict: return "RUNTIME_BP_CONFLICT";
    default: return "NONE";
  }
}

void sendBrakePointConfigError(uint8_t axisIndex,
                               uint8_t requestedSlot,
                               const BrakePointConfigValidation &result) {
  if (currentAckId != nullptr) {
    LinkSerial.print("ACK ");
    LinkSerial.print(currentAckId);
    LinkSerial.print(' ');
  }
  LinkSerial.print("ERR BP_CONFIG J=");
  LinkSerial.print(axisIndex + 1);
  LinkSerial.print(" SLOT=");
  if (requestedSlot == BP_INVALID_BOUNDARY) {
    LinkSerial.print("ALL");
  } else {
    LinkSerial.print(requestedSlot + 1);
  }
  LinkSerial.print(" REASON=");
  LinkSerial.print(brakePointConfigErrorName(result.error));
  if (result.boundary != BP_INVALID_BOUNDARY) {
    LinkSerial.print(" BOUNDARY=");
    printBrakePointBoundaryId(result.boundary);
  }
  if (result.otherBoundary != BP_INVALID_BOUNDARY) {
    LinkSerial.print(" OTHER=");
    printBrakePointBoundaryId(result.otherBoundary);
  }
  LinkSerial.println();
}

bool brakePointConfigurationMayChange(
    uint8_t axisIndex,
    BrakePointConfigValidation &result) {
  if (motors[axisIndex].direction != Direction::Stop ||
      motors[axisIndex].pwm != 0 ||
      motors[axisIndex].braking ||
      digitalRead(motorPins[axisIndex].en) == HIGH) {
    result.error = BrakePointConfigError::MotorActive;
    return false;
  }
  if (targets[axisIndex].active) {
    result.error = BrakePointConfigError::TargetActive;
    return false;
  }
  if (axisRuntime[axisIndex].brakePointPhase != BrakePointPhase::Inactive) {
    result.error = BrakePointConfigError::RuntimeActive;
    return false;
  }
  return true;
}

bool validateBrakePointConfiguration(
    uint8_t axisIndex,
    const BrakePointSlot slots[BP_SLOT_COUNT],
    BrakePointConfigValidation &result) {
  result = {};
  const float tolerance =
      fmaxf(settings.axis[axisIndex].toleranceDeg,
            BP_MIN_RECOVERY_TOLERANCE_DEG);
  const float clearance = configuredBrakePointClearanceDeg(axisIndex);

  for (uint8_t slot = 0; slot < BP_SLOT_COUNT; ++slot) {
    const BrakePointMode modeValue =
        static_cast<BrakePointMode>(slots[slot].mode);
    if (modeValue > BrakePointMode::BlockDecrease) {
      result.error = BrakePointConfigError::BadMode;
      result.boundary = slot;
      return false;
    }
    if (modeValue != BrakePointMode::Disabled &&
        !isfinite(slots[slot].angleDeg)) {
      result.error = BrakePointConfigError::BadAngle;
      result.boundary = slot;
      return false;
    }
  }

  for (uint8_t first = 0; first < BP_BOUNDARY_COUNT; ++first) {
    BrakePointBoundaryView firstBoundary;
    if (!brakePointBoundaryFromConfig(
            slots, first, firstBoundary)) {
      continue;
    }
    for (uint8_t second = first + 1;
         second < BP_BOUNDARY_COUNT;
         ++second) {
      BrakePointBoundaryView secondBoundary;
      if (!brakePointBoundaryFromConfig(
              slots, second, secondBoundary)) {
        continue;
      }
      if (fabsf(bpShortestErrorDeg(firstBoundary.angleDeg,
                                   secondBoundary.angleDeg)) <= tolerance) {
        result.error = BrakePointConfigError::DuplicateBoundary;
        result.boundary = first;
        result.otherBoundary = second;
        return false;
      }
    }
  }

  for (uint8_t sourceIndex = 0;
       sourceIndex < BP_BOUNDARY_COUNT;
       ++sourceIndex) {
    BrakePointBoundaryView source;
    if (!brakePointBoundaryFromConfig(
            slots, sourceIndex, source)) {
      continue;
    }
    const BrakePointApproach possibleApproaches[2] = {
        BrakePointApproach::FromBelow,
        BrakePointApproach::FromAbove};
    const uint8_t approachCount =
        source.mode == BrakePointMode::Wall ? 2 : 1;
    for (uint8_t approachIndex = 0;
         approachIndex < approachCount;
         ++approachIndex) {
      const BrakePointApproach approach =
          source.mode == BrakePointMode::Wall
              ? possibleApproaches[approachIndex]
              : bpFixedApproachForMode(source.mode);
      const float overshoot = normalize360(
          source.angleDeg +
          (approach == BrakePointApproach::FromBelow
               ? tolerance
               : -tolerance));
      const float target = normalize360(
          source.angleDeg +
          (approach == BrakePointApproach::FromBelow
               ? -clearance
               : clearance));
      for (uint8_t otherIndex = 0;
           otherIndex < BP_BOUNDARY_COUNT;
           ++otherIndex) {
        if (otherIndex == sourceIndex) {
          continue;
        }
        BrakePointBoundaryView other;
        if (!brakePointBoundaryFromConfig(
                slots, otherIndex, other)) {
          continue;
        }
        if (bpPathCrossesForbidden(
                other.mode, overshoot, target, other.angleDeg)) {
          result.error =
              BrakePointConfigError::RecoveryBlockedByBoundary;
          result.boundary = sourceIndex;
          result.otherBoundary = otherIndex;
          return false;
        }
      }
    }
  }
  return true;
}

void printHelp() {
  sendLine("OK COMMANDS");
  sendLine("mode arm|armtest|drill confirm, mode safe");
  sendLine("get mode|fault|motors|sensors|status|telemetry|as5600 [channel]");
  sendLine("stream as5600 <channel> [interval_ms], stream off");
  sendLine("seq <id> <command>");
  sendLine("save");
  sendLine("forward <j> [pwm], backward <j> [pwm], axis <j> <-1000..1000>, stop [j], stopall");
  sendLine("servo pulse <800..2200>|off; servo forward|backward [1..100]; servo diag");
  sendLine("joy axis <j> <-1000..1000>, joy button <id> <0|1>, joy disconnect");
  sendLine("zero <j>, goto <j> <angle>, rotate|move <j> <angle>");
  sendLine("set <j> invert|maxpwm|posgain|neggain|dead|keybind|negkeybind|as5600|kp|kd|minpwm|tolerance|stopmode <value>");
  sendLine("set <j> bp <1..4> off|wall|block_increase|block_decrease [angle]");
  sendLine("set <j> bpreact|bpdecel|bplead|bpclear|bprecpwm|bprectime <value> (J1..J3)");
  sendLine("set bts4|bts5 sendpwm <0.00..1.00> (elevator only)");
  sendLine("set telemetry <100..60000>");
  sendLine("set servostate <1..5> (1=Final lid, 2=Tube 4, 3=Tube 3, 4=Tube 2, 5=Tube 1)");
  sendLine("set servocal clear (clear five stored servo calibrations)");
  sendLine("set servoangle <1..55>, set servowiggleangle <1..30>, set servospeed <5..120>");
  sendLine("set servo invert|speed <value> (servo gamepad control is disabled)");
  sendLine("servo <1..5> (1=Final lid, 5=Tube 1), servowiggle");
  sendLine("drillled <1..4> <0|1>");
  sendLine("failsafe linkloss|gamepad|estop, estop");
  sendLine("hb (silent compatibility command; no link-timeout action)");
  sendLine("params, factoryreset confirm");
  sendLine("elv up|down|stop|default|invert|maxpwm [value]");
  sendLine("drill dig|extract|stop|default|invert|maxpwm [value]");
}

void printServoTimerDiagnostics() {
  const ServoPwmDiagnostics diag = servoPwmDiagnostics();
  LinkSerial.print("SERVOTIMER PWM:");
  LinkSerial.print(diag.outputEnabled ? 1 : 0);
  LinkSerial.print(" VALID:");
  LinkSerial.print(diag.configurationValid ? 1 : 0);
  LinkSerial.print(" CLK:");
  LinkSerial.print(diag.timerClockHz);
  LinkSerial.print(" PSC:");
  LinkSerial.print(diag.prescaler);
  LinkSerial.print(" ARR:");
  LinkSerial.print(diag.autoReload);
  LinkSerial.print(" CCR1:");
  LinkSerial.print(diag.compare);
  LinkSerial.print(" CNT:");
  LinkSerial.print(diag.counter);
  LinkSerial.print(" DIER:");
  LinkSerial.print(diag.interruptEnable);
  LinkSerial.print(" PA0MODE:");
  LinkSerial.print(diag.gpioMode);
  LinkSerial.print(" PA0AF:");
  LinkSerial.print(diag.gpioAlternateFunction);
  LinkSerial.print(" PULSE:");
  LinkSerial.print(diag.commandedPulseUs);
  LinkSerial.print(" WRITES:");
  LinkSerial.print(diag.writeCount);
  LinkSerial.print(" LASTCHANGE_MS:");
  LinkSerial.print(diag.lastChangeMs);
  LinkSerial.print(" SOURCE:");
  LinkSerial.println(servoPulseSourceName(diag.lastSource));
}

void printServoExtendedTelemetry() {
  LinkSerial.print("SERVOREFX VALID:");
  LinkSerial.print(servoRuntime.stateInitialized ? 1 : 0);
  LinkSerial.print(" CURRENTSTATE:");
  if (servoRuntime.currentState >= SERVO_STATE_MIN &&
      servoRuntime.currentState <= SERVO_STATE_MAX) {
    LinkSerial.print(servoRuntime.currentState);
  } else {
    LinkSerial.print("NA");
  }
  LinkSerial.print(" TARGETSTATE:");
  if (servoRuntime.targetState >= SERVO_STATE_MIN &&
      servoRuntime.targetState <= SERVO_STATE_MAX) {
    LinkSerial.print(servoRuntime.targetState);
  } else {
    LinkSerial.print("NA");
  }
  LinkSerial.print(" CURRENTPULSE:");
  LinkSerial.print(servoRuntime.currentPulseUs);
  LinkSerial.print(" REFERENCEPULSE:");
  LinkSerial.print(servoRuntime.referencePulseUs);
  LinkSerial.print(" TARGETPULSE:");
  LinkSerial.print(servoRuntime.targetPulseUs);
  LinkSerial.print(" ERROR:");
  LinkSerial.println(servoErrorName(servoRuntime.error));

  LinkSerial.print("SERVOMANUAL VALUE:");
  LinkSerial.print(0);
  LinkSerial.print(" ENABLED:");
  LinkSerial.print(0);
  LinkSerial.print(" ACTIVE:");
  // Kept in telemetry for protocol compatibility. Manual/gamepad servo
  // control was deliberately removed, so this value is always inactive.
  LinkSerial.print(0);
  LinkSerial.print(" NEUTRAL:");
  LinkSerial.print(0);
  LinkSerial.print(" CONFIGSPEED:");
  LinkSerial.print(settings.servo.speedDegPerSec);
  LinkSerial.print(" ACTIVESPEED:");
  LinkSerial.print(servoActiveSpeedDegPerSec());
  LinkSerial.print(" ANGLESTEP:");
  LinkSerial.print(settings.servo.stateAngleDeg);
  LinkSerial.print(" INVERT:");
  LinkSerial.print(settings.servo.invert);
  LinkSerial.print(" PWM:");
  LinkSerial.println(servoRuntime.outputEnabled ? 1 : 0);

  LinkSerial.print("SERVOANGLES VALID:");
  LinkSerial.print(servoRuntime.stateInitialized ? 1 : 0);
  LinkSerial.print(" CURRENT:");
  if (servoRuntime.stateInitialized) {
    const float currentAngle =
        (servoAngleForPulse(static_cast<float>(servoRuntime.currentPulseUs)) -
         servoAngleForPulse(static_cast<float>(servoRuntime.referencePulseUs))) *
        (settings.servo.invert ? -1.0f : 1.0f);
    LinkSerial.print(currentAngle, 1);
  } else {
    LinkSerial.print("NA");
  }
  LinkSerial.print(" TARGET:");
  if (servoRuntime.stateInitialized) {
    const float targetAngle =
        (servoAngleForPulse(static_cast<float>(servoRuntime.targetPulseUs)) -
         servoAngleForPulse(static_cast<float>(servoRuntime.referencePulseUs))) *
        (settings.servo.invert ? -1.0f : 1.0f);
    LinkSerial.print(targetAngle, 1);
  } else {
    LinkSerial.print("NA");
  }
  LinkSerial.print(" REFERENCE:");
  if (servoRuntime.stateInitialized) {
    LinkSerial.println("0.0");
  } else {
    LinkSerial.println("NA");
  }

  LinkSerial.print("SERVOCAL MASK:");
  LinkSerial.print(settings.servo.statePulseValidMask);
  for (uint8_t state = SERVO_STATE_MIN;
       state <= SERVO_STATE_MAX;
       ++state) {
    LinkSerial.print(" P");
    LinkSerial.print(state);
    LinkSerial.print(':');
    if (servoStatePulseCalibrated(state)) {
      LinkSerial.print(
          settings.servo.statePulseUs[state - SERVO_STATE_MIN]);
    } else {
      LinkSerial.print("NA");
    }
  }
  LinkSerial.println();
}

void printSendPwmPercent(uint8_t percent) {
  LinkSerial.print(percent / SENDPWM_PERCENT_MAX);
  LinkSerial.print('.');
  const uint8_t fractional = percent % SENDPWM_PERCENT_MAX;
  if (fractional < 10) {
    LinkSerial.print('0');
  }
  LinkSerial.print(fractional);
}

void printParams() {
  LinkSerial.print("OK MODE ");
  LinkSerial.println(modeName(mode));
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    const AxisConfig &cfg = settings.axis[i];
    LinkSerial.print("J");
    LinkSerial.print(i + 1);
    LinkSerial.print(" INVERT=");
    LinkSerial.print(cfg.invert);
    LinkSerial.print(" MAXPWM=");
    LinkSerial.print(cfg.maxPwm);
    LinkSerial.print(" DEFAULT=");
    LinkSerial.print(cfg.defaultPwm);
    LinkSerial.print(" POSGAIN=");
    LinkSerial.print(cfg.posGain);
    LinkSerial.print(" NEGGAIN=");
    LinkSerial.print(cfg.negGain);
    LinkSerial.print(" DEAD=");
    LinkSerial.print(cfg.dead);
    LinkSerial.print(" STOPMODE=");
    LinkSerial.print(stopModeName(cfg.stopMode));
    LinkSerial.print(" KP=");
    LinkSerial.print(cfg.kp);
    LinkSerial.print(" KD=");
    LinkSerial.print(cfg.kd);
    LinkSerial.print(" MINPWM=");
    LinkSerial.print(cfg.minPwm);
    LinkSerial.print(" TOL=");
    LinkSerial.print(cfg.toleranceDeg, 2);
    if (i < 3) {
      const BrakePointTuning &bp = settings.brakePoint[i];
      LinkSerial.print(" BPREACT_MS=");
      LinkSerial.print(bp.reactionMs);
      LinkSerial.print(" BPDECEL_DPS2=");
      LinkSerial.print(bp.decelDps2);
      LinkSerial.print(" BPLEAD_DEG=");
      LinkSerial.print(bp.leadMarginDeg, 2);
      LinkSerial.print(" BPCLEAR_DEG=");
      LinkSerial.print(bp.recoveryClearanceDeg, 2);
      LinkSerial.print(" BPRECPWM=");
      LinkSerial.print(bp.recoveryMaxPwm);
      LinkSerial.print(" BPRECTIME_MS=");
      LinkSerial.print(bp.recoveryTimeoutMs);
    }
    LinkSerial.print(" KEYFWD=");
    LinkSerial.print(cfg.keybindForward);
    LinkSerial.print(" KEYBACK=");
    LinkSerial.print(cfg.keybindBackward);
    LinkSerial.print(" AS5600=");
    const int8_t channel = configuredAS5600Channel(i);
    if (channel < 0) {
      LinkSerial.println("OFF");
    } else {
      LinkSerial.println(channel);
    }
    if (i < 3) {
      for (uint8_t slot = 0; slot < BP_SLOT_COUNT; ++slot) {
        const BrakePointSlot &bpSlot =
            settings.brakePointSlots[i][slot];
        LinkSerial.print("BP J");
        LinkSerial.print(i + 1);
        LinkSerial.print(" SLOT=");
        LinkSerial.print(slot + 1);
        LinkSerial.print(" MODE=");
        const BrakePointMode slotMode =
            static_cast<BrakePointMode>(bpSlot.mode);
        LinkSerial.print(brakePointModeName(slotMode));
        LinkSerial.print(" ANGLE=");
        if (slotMode == BrakePointMode::Disabled) {
          LinkSerial.println("NA");
        } else {
          LinkSerial.println(normalize360(bpSlot.angleDeg), 2);
        }
      }
    }
  }
  LinkSerial.print("ELV INVERT=");
  LinkSerial.print(settings.elevator.invert);
  LinkSerial.print(" MAXPWM=");
  LinkSerial.print(settings.elevator.maxPwm);
  LinkSerial.print(" DEFAULT=");
  LinkSerial.print(settings.elevator.defaultPwm);
  LinkSerial.print(" SENDPWM_BTS4=");
  printSendPwmPercent(settings.elevator.sendPwmPercent[0]);
  LinkSerial.print(" SENDPWM_BTS5=");
  printSendPwmPercent(settings.elevator.sendPwmPercent[1]);
  LinkSerial.println();
  LinkSerial.print("DRILL INVERT=");
  LinkSerial.print(settings.drill.invert);
  LinkSerial.print(" MAXPWM=");
  LinkSerial.print(settings.drill.maxPwm);
  LinkSerial.print(" DEFAULT=");
  LinkSerial.println(settings.drill.defaultPwm);
  LinkSerial.print("SERVO INIT=");
  LinkSerial.print(servoRuntime.stateInitialized ? 1 : 0);
  LinkSerial.print(" STATE=");
  if (servoRuntime.currentState >= SERVO_STATE_MIN &&
      servoRuntime.currentState <= SERVO_STATE_MAX) {
    LinkSerial.print(servoRuntime.currentState);
  } else {
    LinkSerial.print("NA");
  }
  LinkSerial.print(" ANGLE=");
  LinkSerial.print(settings.servo.stateAngleDeg);
  LinkSerial.print(" WIGGLEANGLE=");
  LinkSerial.print(settings.servo.wiggleAngleDeg);
  LinkSerial.print(" INVERT=");
  LinkSerial.print(settings.servo.invert);
  LinkSerial.print(" SPEED=");
  LinkSerial.print(settings.servo.speedDegPerSec);
  LinkSerial.print(" KEYNEXT=");
  LinkSerial.print(settings.servo.keybindNext);
  LinkSerial.print(" KEYPREV=");
  LinkSerial.println(settings.servo.keybindPrevious);
  LinkSerial.print("SERVOREF STATE=");
  if (servoRuntime.stateInitialized) {
    LinkSerial.print(servoRuntime.referenceState);
  } else {
    LinkSerial.print("NA");
  }
  LinkSerial.print(" PULSE=");
  LinkSerial.print(servoRuntime.referencePulseUs);
  LinkSerial.print(" ASSUMED=");
  LinkSerial.print(servoRuntime.referencePulseAssumed ? 1 : 0);
  LinkSerial.print(" PWM=");
  LinkSerial.println(servoRuntime.outputEnabled ? 1 : 0);
  printServoExtendedTelemetry();
  LinkSerial.print("TELEMETRY_MS=");
  LinkSerial.println(statusIntervalMs);
}

void printAS5600Channel(uint8_t channel, bool &foundAny) {
  uint16_t raw = 0;
  float angle = 0.0f;
  bool usedFastLoopSample = false;
  for (uint8_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
    if (configuredAS5600Channel(axisIndex) != static_cast<int8_t>(channel)) {
      continue;
    }
    float correctedAngleDeg = 0.0f;
    if (getFreshAxisAngle(axisIndex, correctedAngleDeg)) {
      raw = axisRuntime[axisIndex].rawAngle;
      angle = axisRuntime[axisIndex].sensorAngleDeg;
      usedFastLoopSample = true;
      break;
    }
  }
  if (!usedFastLoopSample && !sensors.readAS5600Angle(channel, raw, angle)) {
    return;
  }

  uint8_t status = 0;
  const bool hasStatus = sensors.readAS5600Status(channel, status);
  foundAny = true;

  LinkSerial.print("AS5600 CH=");
  LinkSerial.print(channel);
  LinkSerial.print(" RAW=");
  LinkSerial.print(raw);
  LinkSerial.print(" DEG=");
  LinkSerial.print(angle, 2);
  if (hasStatus) {
    LinkSerial.print(" STATUS=0x");
    if (status < 16) {
      LinkSerial.print('0');
    }
    LinkSerial.print(status, HEX);
    LinkSerial.print(" ");
    LinkSerial.print(as5600MagnetStatusName(status));
  } else {
    LinkSerial.print(" STATUS=READ_FAIL");
  }
  LinkSerial.println();
}

void printModeStatus() {
  LinkSerial.print("OK MODE ");
  LinkSerial.println(modeName(mode));
}

void printTelemetryInterval() {
  if (currentAckId != nullptr) {
    LinkSerial.print("ACK ");
    LinkSerial.print(currentAckId);
    LinkSerial.print(' ');
  }
  LinkSerial.print("OK TELEMETRY_MS=");
  LinkSerial.println(statusIntervalMs);
}

void printFaultStatus() {
  LinkSerial.print("OK FAULT SENSOR_BITS=0x");
  LinkSerial.print(sensorFaultBits, HEX);
  LinkSerial.print(" BRAKEPOINT_BITS=0x");
  LinkSerial.print(brakePointFaultBits, HEX);
  LinkSerial.print(" PENDING=");
  LinkSerial.print(static_cast<uint8_t>(pendingConfirm));
  LinkSerial.print(" STREAM_AS5600=");
  LinkSerial.print(streamAS5600Active ? 1 : 0);
  LinkSerial.print(" SETTINGS_DIRTY=");
  LinkSerial.print(settingsDirty ? 1 : 0);
  LinkSerial.print(" SETTINGS_FLASH=");
  LinkSerial.print(settingsLoadedFromFlash ? 1 : 0);
  LinkSerial.print(" I2C_ERRORS=");
  LinkSerial.print(i2c_error_count);
  LinkSerial.print(" I2C_RECOVERIES=");
  LinkSerial.print(i2c_recovery_count);
  for (uint8_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
    LinkSerial.print(" J");
    LinkSerial.print(axisIndex + 1);
    LinkSerial.print("_BP_ACTIVE=");
    if (axisRuntime[axisIndex].activeBrakePointBoundary ==
        BP_INVALID_BOUNDARY) {
      LinkSerial.print("NONE");
    } else {
      printBrakePointBoundaryId(
          axisRuntime[axisIndex].activeBrakePointBoundary);
    }
    LinkSerial.print(" J");
    LinkSerial.print(axisIndex + 1);
    LinkSerial.print("_BP_REASON=");
    LinkSerial.print(brakePointFaultReasonName(
        axisRuntime[axisIndex].brakePointFaultReason));
  }
  LinkSerial.println();
}

void printMotorStatus() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    LinkSerial.print("M");
    LinkSerial.print(i + 1);
    LinkSerial.print(" DIR=");
    LinkSerial.print(directionName(motors[i].direction));
    LinkSerial.print(" PWM=");
    LinkSerial.print(motors[i].pwm);
    LinkSerial.print(" BRAKE=");
    LinkSerial.print(motors[i].braking ? 1 : 0);
    LinkSerial.print(" EN=");
    LinkSerial.println(digitalRead(motorPins[i].en));
  }
  LinkSerial.print("ELV_BTS4 REQUESTED_PWM=");
  LinkSerial.print(elevatorPwmRuntime.requested[0]);
  LinkSerial.print(" ACTUAL_PWM=");
  LinkSerial.println(elevatorPwmRuntime.actual[0]);
  LinkSerial.print("ELV_BTS5 REQUESTED_PWM=");
  LinkSerial.print(elevatorPwmRuntime.requested[1]);
  LinkSerial.print(" ACTUAL_PWM=");
  LinkSerial.println(elevatorPwmRuntime.actual[1]);
}

void printSensorStatus() {
  for (uint8_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
    const int8_t channel = configuredAS5600Channel(axisIndex);
    float angleDeg = 0.0f;
    const bool ok = getFreshAxisAngle(axisIndex, angleDeg);
    LinkSerial.print("SENSOR J");
    LinkSerial.print(axisIndex + 1);
    LinkSerial.print(" CH=");
    if (channel < 0) {
      LinkSerial.print("OFF");
    } else {
      LinkSerial.print(channel);
    }
    if (!ok) {
      LinkSerial.println(" ERR=READ_FAIL");
      continue;
    }

    uint8_t status = 0;
    const bool hasStatus = channel >= 0 && sensors.readAS5600Status(static_cast<uint8_t>(channel), status);
    LinkSerial.print(" DEG=");
    LinkSerial.print(angleDeg, 2);
    LinkSerial.print(" VEL_DPS=");
    LinkSerial.print(axisRuntime[axisIndex].velocityDps, 2);
    if (hasStatus) {
      LinkSerial.print(" STATUS=0x");
      if (status < 16) {
        LinkSerial.print('0');
      }
      LinkSerial.print(status, HEX);
      LinkSerial.print(" ");
      LinkSerial.print(as5600MagnetStatusName(status));
    }
    LinkSerial.println();
  }
}

struct AxisTelemetrySnapshot {
  bool angleValid;
  float angleDeg;
  float velocityDps;
};

void printAxisTelemetry(uint8_t motorIndex, const AxisTelemetrySnapshot snapshots[3]) {
  LinkSerial.print(" J");
  LinkSerial.print(motorIndex + 1);
  LinkSerial.print("_PWM:");
  LinkSerial.print(motors[motorIndex].braking ? MAX_PWM : motors[motorIndex].pwm);
  LinkSerial.print(" J");
  LinkSerial.print(motorIndex + 1);
  LinkSerial.print("_DIR:");
  LinkSerial.print(directionNameLower(motors[motorIndex].direction));
  LinkSerial.print(" J");
  LinkSerial.print(motorIndex + 1);
  LinkSerial.print("_EN:");
  LinkSerial.print(digitalRead(motorPins[motorIndex].en) == HIGH ? 1 : 0);
  LinkSerial.print(" J");
  LinkSerial.print(motorIndex + 1);
  LinkSerial.print("_ANGLE:");
  if (motorIndex < 3 && snapshots[motorIndex].angleValid) {
    LinkSerial.print(snapshots[motorIndex].angleDeg, 2);
  } else {
    LinkSerial.print("NA");
  }
  LinkSerial.print(" J");
  LinkSerial.print(motorIndex + 1);
  LinkSerial.print("_VEL_DPS:");
  if (motorIndex < 3 && snapshots[motorIndex].angleValid) {
    LinkSerial.print(snapshots[motorIndex].velocityDps, 2);
  } else {
    LinkSerial.print("NA");
  }
}

void printStatusTelemetry() {
  AxisTelemetrySnapshot snapshots[3];
  for (uint8_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
    snapshots[axisIndex].angleValid = axisRuntime[axisIndex].angleValid;
    snapshots[axisIndex].angleDeg = axisRuntime[axisIndex].angleDeg;
    snapshots[axisIndex].velocityDps = axisRuntime[axisIndex].velocityDps;
  }

  LinkSerial.print("MODE:");
  LinkSerial.print(modeNameLower(mode));
  LinkSerial.print(" STOPMODE:");
  LinkSerial.print(telemetryStopModeName());
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    printAxisTelemetry(i, snapshots);
  }
  LinkSerial.println();
}

void printDrillAccessoryTelemetry() {
  LinkSerial.print("SERVO INIT:");
  LinkSerial.print(servoRuntime.stateInitialized ? 1 : 0);
  LinkSerial.print(" STATE:");
  if (servoRuntime.currentState >= SERVO_STATE_MIN &&
      servoRuntime.currentState <= SERVO_STATE_MAX) {
    LinkSerial.print(servoRuntime.currentState);
  } else {
    LinkSerial.print("NA");
  }
  LinkSerial.print(" TARGET:");
  if (servoRuntime.targetState >= SERVO_STATE_MIN &&
      servoRuntime.targetState <= SERVO_STATE_MAX) {
    LinkSerial.print(servoRuntime.targetState);
  } else {
    LinkSerial.print("NA");
  }
  LinkSerial.print(" MOVING:");
  LinkSerial.print(servoRuntime.motion != ServoMotion::Idle ? 1 : 0);
  LinkSerial.print(" ANGLE:");
  LinkSerial.print(settings.servo.stateAngleDeg);
  LinkSerial.print(" WIGGLE:");
  LinkSerial.print(settings.servo.wiggleAngleDeg);
  LinkSerial.print(" INVERT:");
  LinkSerial.print(settings.servo.invert);
  LinkSerial.print(" SPEED:");
  LinkSerial.print(settings.servo.speedDegPerSec);
  LinkSerial.print(" PWM:");
  LinkSerial.print(servoRuntime.outputEnabled ? 1 : 0);
  LinkSerial.print(" PULSE:");
  LinkSerial.print(servoRuntime.currentPulseUs);
  LinkSerial.print(" TARGETPULSE:");
  LinkSerial.print(servoRuntime.targetPulseUs);
  LinkSerial.print(" SAFETY:");
  LinkSerial.print(servoSafetyReasonName(servoRuntime.safetyReason));
  LinkSerial.print(" STATE_NAME:");
  LinkSerial.print(servoStateName(servoRuntime.currentState));
  LinkSerial.print(" TARGET_NAME:");
  LinkSerial.println(servoStateName(servoRuntime.targetState));
  printServoExtendedTelemetry();

  LinkSerial.print("DRILLLEDS");
  for (uint8_t i = 0; i < DRILL_LED_COUNT; ++i) {
    LinkSerial.print(" LED");
    LinkSerial.print(i + 1);
    LinkSerial.print(':');
    LinkSerial.print(drillLedStates[i] ? 1 : 0);
  }
  LinkSerial.println();
}

void handleGetCommand(char **tokens, uint8_t count) {
  if (count < 2) {
    sendErr("USAGE_GET");
    return;
  }

  if (strcmp(tokens[1], "mode") == 0) {
    printModeStatus();
    return;
  }

  if (strcmp(tokens[1], "fault") == 0) {
    printFaultStatus();
    return;
  }

  if (strcmp(tokens[1], "motors") == 0) {
    printMotorStatus();
    return;
  }

  if (strcmp(tokens[1], "sensors") == 0) {
    printSensorStatus();
    return;
  }

  if (strcmp(tokens[1], "status") == 0) {
    printStatusTelemetry();
    printDrillAccessoryTelemetry();
    return;
  }

  if (strcmp(tokens[1], "telemetry") == 0) {
    printTelemetryInterval();
    return;
  }

  if (strcmp(tokens[1], "as5600") != 0) {
    sendErr("BAD_GET_TARGET");
    return;
  }

  bool foundAny = false;
  if (count >= 3) {
    int channel = 0;
    if (!parseIntToken(tokens[2], channel) || !inRange(channel, 0, 7)) {
      sendErr("BAD_CHANNEL");
      return;
    }
    printAS5600Channel(static_cast<uint8_t>(channel), foundAny);
  } else {
    for (uint8_t channel = 0; channel < 8; ++channel) {
      printAS5600Channel(channel, foundAny);
    }
  }

  if (!foundAny) {
    sendErr("AS5600_NOT_FOUND");
  }
}

void handleSaveCommand() {
  if (mode != SystemMode::Safe) {
    sendErr("SAVE_REQUIRES_SAFE");
    return;
  }
  if (anyMotorOutputActive()) {
    sendErr("SAVE_REQUIRES_STOPPED");
    return;
  }

  sendOk("SAVE_BEGIN");
  LinkSerial.flush();
  persistSettingsToFlash();
  sendOk("SAVED_FLASH");
}

void handleStreamCommand(char **tokens, uint8_t count) {
  if (count < 2) {
    sendErr("USAGE_STREAM");
    return;
  }

  if (strcmp(tokens[1], "off") == 0 || strcmp(tokens[1], "stop") == 0) {
    streamAS5600Active = false;
    sendOk("STREAM_OFF");
    return;
  }

  if (strcmp(tokens[1], "as5600") != 0 || count < 3) {
    sendErr("USAGE_STREAM_AS5600");
    return;
  }

  int channel = 0;
  if (!parseIntToken(tokens[2], channel) || !inRange(channel, 0, 7)) {
    sendErr("BAD_CHANNEL");
    return;
  }

  int intervalMs = DEFAULT_STREAM_MS;
  if (count >= 4 && (!parseIntToken(tokens[3], intervalMs) || !inRange(intervalMs, 50, 5000))) {
    sendErr("BAD_INTERVAL");
    return;
  }

  streamAS5600Channel = static_cast<uint8_t>(channel);
  streamIntervalMs = static_cast<uint16_t>(intervalMs);
  lastStreamMs = 0;
  streamAS5600Active = true;
  sendOk("STREAM_AS5600");
}

void handleModeCommand(char **tokens, uint8_t count) {
  if (count < 2) {
    LinkSerial.print("OK MODE ");
    LinkSerial.println(modeName(mode));
    return;
  }

  if (strcmp(tokens[1], "safe") == 0 || strcmp(tokens[1], "none") == 0) {
    enterMode(SystemMode::Safe);
    return;
  }

  const bool confirmed = count >= 3 && strcmp(tokens[2], "confirm") == 0;
  if (strcmp(tokens[1], "arm") == 0) {
    if (confirmed) {
      enterMode(SystemMode::Arm);
    } else {
      pendingConfirm = ConfirmAction::ModeArm;
      sendLine("CONFIRM MODE ARM? Y/N");
    }
    return;
  }

  if (strcmp(tokens[1], "armtest") == 0) {
    if (confirmed) {
      enterMode(SystemMode::ArmTest);
    } else {
      pendingConfirm = ConfirmAction::ModeArmTest;
      sendLine("CONFIRM MODE ARMTEST? BP/GOTO/HOLD BYPASSED, MANUAL ONLY. Y/N");
    }
    return;
  }

  if (strcmp(tokens[1], "drill") == 0) {
    if (confirmed) {
      enterMode(SystemMode::Drill);
    } else {
      pendingConfirm = ConfirmAction::ModeDrill;
      sendLine("CONFIRM MODE DRILL? Y/N");
    }
    return;
  }

  sendErr("BAD_MODE");
}

void handleSetCommand(char **tokens, uint8_t count) {
  if (count >= 3 && strcmp(tokens[1], "telemetry") == 0) {
    int telemetryMs = 0;
    if (!parseIntToken(tokens[2], telemetryMs) ||
        !inRange(telemetryMs, MIN_STATUS_MS, MAX_STATUS_MS)) {
      sendErr("BAD_TELEMETRY_INTERVAL");
      return;
    }
    statusIntervalMs = static_cast<uint16_t>(telemetryMs);
    lastStatusMs = millis();
    printTelemetryInterval();
    return;
  }

  if (count >= 3 && strcmp(tokens[1], "servopulse") == 0) {
    if (mode != SystemMode::Drill) {
      sendErr("MODE_NOT_DRILL");
      return;
    }
    int pulseUs = 0;
    if (!parseIntToken(tokens[2], pulseUs) ||
        !inRange(pulseUs, SERVO_MIN_US, SERVO_MAX_US)) {
      sendErr("BAD_SERVO_PULSE_800_2200");
      return;
    }
    if (servoRuntime.motion != ServoMotion::Idle) {
      sendErr("SERVO_BUSY");
      return;
    }
    if (servoRuntime.outputEnabled) {
      sendErr("SERVO_PWM_ACTIVE");
      return;
    }
    // Open-loop 3-wire servos provide no position feedback. This command does
    // not enable PWM or move the servo; it explicitly declares the pulse that
    // corresponds to the present, externally established physical position.
    servoRuntime.currentPulseUs = static_cast<uint16_t>(pulseUs);
    servoRuntime.targetPulseUs = static_cast<uint16_t>(pulseUs);
    servoRuntime.referencePulseUs = static_cast<uint16_t>(pulseUs);
    servoRuntime.homePulseUs = static_cast<uint16_t>(pulseUs);
    servoRuntime.rampPulseUs = static_cast<float>(pulseUs);
    servoRuntime.pulseReferenceKnown = true;
    servoRuntime.referencePulseAssumed = false;
    servoRuntime.stateInitialized = false;
    servoRuntime.currentState = 0;
    servoRuntime.targetState = 0;
    servoRuntime.referenceState = 0;
    servoRuntime.safetyReason = ServoSafetyReason::Ready;
    servoRuntime.error = ServoError::None;
    sendOk("SERVOPULSE_DECLARED_NO_MOVE");
    return;
  }

  if (count >= 3 && strcmp(tokens[1], "servostate") == 0) {
    if (mode != SystemMode::Drill) {
      sendErr("MODE_NOT_DRILL");
      return;
    }
    int state = 0;
    if (!parseIntToken(tokens[2], state) ||
        !inRange(state, SERVO_STATE_MIN, SERVO_STATE_MAX)) {
      sendErr("BAD_SERVO_STATE");
      return;
    }
    if (!servoRuntime.pulseReferenceKnown) {
      servoRuntime.error = ServoError::CurrentPulseUnknown;
      sendErr("SERVO_POSITION_UNKNOWN_APPLY_FIXED_PULSE");
      return;
    }
    if (servoRuntime.motion != ServoMotion::Idle) {
      sendErr("SERVO_BUSY");
      return;
    }
    const uint8_t calibratedState = static_cast<uint8_t>(state);
    if (!servoCalibrationPulseFits(calibratedState,
                                   servoRuntime.currentPulseUs)) {
      servoRuntime.safetyReason = ServoSafetyReason::PulseRange;
      servoRuntime.error = ServoError::TargetPulseRange;
      sendErr("SERVO_CALIBRATION_ORDER");
      return;
    }
    settings.servo.statePulseUs[calibratedState - SERVO_STATE_MIN] =
        servoRuntime.currentPulseUs;
    settings.servo.statePulseValidMask |=
        static_cast<uint8_t>(1U << (calibratedState - SERVO_STATE_MIN));
    markSettingsDirty();

    // A calibrated capture owns the servo immediately at the unchanged pulse.
    servoRuntime.motion = ServoMotion::Idle;
    servoRuntime.targetPulseUs = servoRuntime.currentPulseUs;
    servoRuntime.rampPulseUs = servoRuntime.currentPulseUs;
    servoRuntime.referencePulseUs = servoRuntime.currentPulseUs;
    servoRuntime.stateInitialized = true;
    servoRuntime.currentState = calibratedState;
    servoRuntime.targetState = calibratedState;
    servoRuntime.referenceState = calibratedState;
    servoRuntime.referencePulseUs = servoRuntime.currentPulseUs;
    servoRuntime.referencePulseAssumed = false;
    servoRuntime.targetPulseUs = servoRuntime.currentPulseUs;
    servoRuntime.homePulseUs = servoRuntime.currentPulseUs;
    servoRuntime.rampPulseUs = servoRuntime.currentPulseUs;
    servoRuntime.motionStartedMs = 0;
    servoRuntime.lastRampMs = millis();
    servoRuntime.nextStepMs = 0;
    servoRuntime.wiggleDirection = 1;
    servoRuntime.safetyReason = ServoSafetyReason::Ready;
    servoRuntime.error = ServoError::None;
    LinkSerial.print("OK SERVOSTATE_");
    LinkSerial.print(calibratedState);
    LinkSerial.print("_CALIBRATED_NO_MOVE MASK:");
    LinkSerial.println(settings.servo.statePulseValidMask);
    return;
  }

  if (count >= 3 && strcmp(tokens[1], "servocal") == 0) {
    if (strcmp(tokens[2], "clear") != 0) {
      sendErr("USAGE_SET_SERVOCAL_CLEAR");
      return;
    }
    if (servoRuntime.motion != ServoMotion::Idle) {
      sendErr("SERVO_BUSY");
      return;
    }
    memset(settings.servo.statePulseUs,
           0,
           sizeof(settings.servo.statePulseUs));
    settings.servo.statePulseValidMask = 0;
    servoRuntime.stateInitialized = false;
    servoRuntime.currentState = 0;
    servoRuntime.targetState = 0;
    servoRuntime.referenceState = 0;
    servoRuntime.targetPulseUs = servoRuntime.currentPulseUs;
    servoRuntime.safetyReason = ServoSafetyReason::ConfigChanged;
    servoRuntime.error = ServoError::None;
    markSettingsDirty();
    sendOk("SERVO_CALIBRATION_CLEARED");
    return;
  }

  if (count >= 3 && strcmp(tokens[1], "servoangle") == 0) {
    int angle = 0;
    if (!parseIntToken(tokens[2], angle) ||
        !inRange(angle, 1, MAX_SERVO_STATE_ANGLE_DEG)) {
      sendErr("BAD_SERVO_ANGLE_1_55");
      return;
    }
    if (servoRuntime.motion != ServoMotion::Idle) {
      sendErr("SERVO_BUSY");
      return;
    }
    const uint8_t newAngle = static_cast<uint8_t>(angle);
    if (settings.servo.stateAngleDeg != newAngle) {
      settings.servo.stateAngleDeg = newAngle;
      servoRuntime.currentState =
          servoRuntime.stateInitialized &&
                  servoRuntime.currentPulseUs == servoRuntime.referencePulseUs
              ? servoRuntime.referenceState
              : 0;
      servoRuntime.targetState = servoRuntime.currentState;
      servoRuntime.targetPulseUs = servoRuntime.currentPulseUs;
      servoRuntime.safetyReason = ServoSafetyReason::ConfigChanged;
      servoRuntime.error = ServoError::None;
    }
    markSettingsDirty();
    sendOk("UPDATED_RAM");
    return;
  }

  if (count >= 3 && strcmp(tokens[1], "servowiggleangle") == 0) {
    int angle = 0;
    if (!parseIntToken(tokens[2], angle) ||
        !inRange(angle, 1, MAX_SERVO_WIGGLE_ANGLE_DEG)) {
      sendErr("BAD_SERVO_WIGGLE_ANGLE_1_30");
      return;
    }
    if (servoRuntime.motion != ServoMotion::Idle) {
      sendErr("SERVO_BUSY");
      return;
    }
    settings.servo.wiggleAngleDeg = static_cast<uint8_t>(angle);
    markSettingsDirty();
    sendOk("UPDATED_RAM");
    return;
  }

  if (count >= 3 && strcmp(tokens[1], "servospeed") == 0) {
    int speed = 0;
    if (!parseIntToken(tokens[2], speed) ||
        !inRange(speed, MIN_SERVO_SPEED_DPS, MAX_SERVO_SPEED_DPS)) {
      sendErr("BAD_SERVO_SPEED_5_120");
      return;
    }
    if (servoRuntime.motion != ServoMotion::Idle) {
      sendErr("SERVO_BUSY");
      return;
    }
    settings.servo.speedDegPerSec = static_cast<uint8_t>(speed);
    markSettingsDirty();
    LinkSerial.print("OK SERVO_SPEED_SET CONFIGURED=");
    LinkSerial.println(settings.servo.speedDegPerSec);
    return;
  }

  if (count >= 4 && strcmp(tokens[1], "servo") == 0) {
    int value = 0;
    if (strcmp(tokens[2], "invert") == 0) {
      if (!parseIntToken(tokens[3], value) || !inRange(value, 0, 1)) {
        sendErr("BAD_VALUE");
        return;
      }
      if (servoRuntime.motion != ServoMotion::Idle) {
        sendErr("SERVO_BUSY");
        return;
      }
      const uint8_t newInvert = static_cast<uint8_t>(value);
      if (settings.servo.invert != newInvert) {
        settings.servo.invert = newInvert;
        memset(settings.servo.statePulseUs,
               0,
               sizeof(settings.servo.statePulseUs));
        settings.servo.statePulseValidMask = 0;
        servoRuntime.stateInitialized = false;
        servoRuntime.currentState = 0;
        servoRuntime.targetState = 0;
        servoRuntime.referenceState = 0;
        servoRuntime.targetPulseUs = servoRuntime.currentPulseUs;
        servoRuntime.safetyReason = ServoSafetyReason::ConfigChanged;
        servoRuntime.error = ServoError::None;
      }
    } else if (strcmp(tokens[2], "speed") == 0) {
      if (!parseIntToken(tokens[3], value) ||
          !inRange(value, MIN_SERVO_SPEED_DPS, MAX_SERVO_SPEED_DPS)) {
        sendErr("BAD_SERVO_SPEED_5_120");
        return;
      }
      if (servoRuntime.motion != ServoMotion::Idle) {
        sendErr("SERVO_BUSY");
        return;
      }
      settings.servo.speedDegPerSec = static_cast<uint8_t>(value);
      LinkSerial.print("OK SERVO_SPEED_SET CONFIGURED=");
      LinkSerial.println(settings.servo.speedDegPerSec);
      markSettingsDirty();
      return;
    } else if (strcmp(tokens[2], "nextkey") == 0 ||
               strcmp(tokens[2], "keynext") == 0) {
      settings.servo.keybindNext = KEYBIND_DISABLED;
      sendErr("SERVO_GAMEPAD_CONTROL_DISABLED");
      return;
    } else if (strcmp(tokens[2], "prevkey") == 0 ||
               strcmp(tokens[2], "keyprev") == 0) {
      settings.servo.keybindPrevious = KEYBIND_DISABLED;
      sendErr("SERVO_GAMEPAD_CONTROL_DISABLED");
      return;
    } else {
      sendErr("BAD_SERVO_PARAM");
      return;
    }
    markSettingsDirty();
    sendOk("UPDATED_RAM");
    return;
  }

  if (count < 4) {
    sendErr("USAGE_SET");
    return;
  }

  if (strncmp(tokens[1], "bts", 3) == 0) {
    uint8_t btsOneBased = 0;
    if (!parseBts(tokens[1], btsOneBased)) {
      sendErr("BAD_BTS");
      return;
    }
    if (strcmp(tokens[2], "sendpwm") != 0) {
      sendErr("BAD_PARAM");
      return;
    }
    if (btsOneBased != 4 && btsOneBased != 5) {
      sendErr("SENDPWM_ELEVATOR_ONLY_BTS4_BTS5");
      return;
    }
    uint8_t percent = 0;
    if (!parseSendPwmPercent(tokens[3], percent)) {
      sendErr("BAD_VALUE");
      return;
    }
    settings.elevator.sendPwmPercent[btsOneBased - 4] = percent;
    markSettingsDirty();
    sendOk("UPDATED_RAM");
    return;
  }

  uint8_t axisOneBased = 0;
  if (!parseAxis(tokens[1], axisOneBased)) {
    sendErr("BAD_AXIS");
    return;
  }
  const uint8_t axisIndex = axisOneBased - 1;
  AxisConfig &cfg = settings.axis[axisIndex];
  int intValue = 0;
  float floatValue = 0.0f;

  if (strcmp(tokens[2], "invert") == 0) {
    if (!parseIntToken(tokens[3], intValue) || !inRange(intValue, 0, 1)) {
      sendErr("BAD_VALUE");
      return;
    }
    cfg.invert = static_cast<uint8_t>(intValue);
  } else if (strcmp(tokens[2], "maxpwm") == 0) {
    if (!parseIntToken(tokens[3], intValue) || !inRange(intValue, 0, 255)) {
      sendErr("BAD_VALUE");
      return;
    }
    cfg.maxPwm = static_cast<uint8_t>(intValue);
    if (cfg.defaultPwm > cfg.maxPwm) {
      cfg.defaultPwm = cfg.maxPwm;
    }
    if (cfg.minPwm > cfg.maxPwm) {
      cfg.minPwm = cfg.maxPwm;
    }
  } else if (strcmp(tokens[2], "default") == 0) {
    if (!parseIntToken(tokens[3], intValue) || !inRange(intValue, 0, 255)) {
      sendErr("BAD_VALUE");
      return;
    }
    cfg.defaultPwm = clampPwm(intValue, cfg.maxPwm);
  } else if (strcmp(tokens[2], "posgain") == 0) {
    if (!parseIntToken(tokens[3], intValue) || !inRange(intValue, 100, 3000)) {
      sendErr("BAD_VALUE");
      return;
    }
    cfg.posGain = static_cast<uint16_t>(intValue);
  } else if (strcmp(tokens[2], "neggain") == 0) {
    if (!parseIntToken(tokens[3], intValue) || !inRange(intValue, 100, 3000)) {
      sendErr("BAD_VALUE");
      return;
    }
    cfg.negGain = static_cast<uint16_t>(intValue);
  } else if (strcmp(tokens[2], "dead") == 0) {
    if (!parseIntToken(tokens[3], intValue) || !inRange(intValue, 0, 400)) {
      sendErr("BAD_VALUE");
      return;
    }
    cfg.dead = static_cast<uint16_t>(intValue);
  } else if (strcmp(tokens[2], "keybind") == 0 || strcmp(tokens[2], "keyfwd") == 0 ||
             strcmp(tokens[2], "keyforward") == 0) {
    if (!parseIntToken(tokens[3], intValue) || !inRange(intValue, KEYBIND_DISABLED, 32767)) {
      sendErr("BAD_VALUE");
      return;
    }
    cfg.keybindForward = static_cast<int16_t>(intValue);
    clearAxisKeybindRuntime(axisIndex);
  } else if (strcmp(tokens[2], "negkeybind") == 0 || strcmp(tokens[2], "keyback") == 0 ||
             strcmp(tokens[2], "keybackward") == 0) {
    if (!parseIntToken(tokens[3], intValue) || !inRange(intValue, KEYBIND_DISABLED, 32767)) {
      sendErr("BAD_VALUE");
      return;
    }
    cfg.keybindBackward = static_cast<int16_t>(intValue);
    clearAxisKeybindRuntime(axisIndex);
  } else if (strcmp(tokens[2], "as5600") == 0) {
    if (axisIndex > 2) {
      sendErr("POSITION_ONLY_J1_J3");
      return;
    }
    if (!parseIntToken(tokens[3], intValue) || !inRange(intValue, -1, 7)) {
      sendErr("BAD_VALUE");
      return;
    }
    coastPositionControllerAxis(axisIndex);
    resetPositionControllerRuntime(axisIndex);
    cfg.as5600Channel = intValue < 0 ? AS5600_CHANNEL_DISABLED : static_cast<uint8_t>(intValue);
    axisRuntime[axisIndex].angleValid = false;
    axisRuntime[axisIndex].hasPreviousAngle = false;
    resetBrakePointRuntime(axisIndex);
    targets[axisIndex].active = false;
    targets[axisIndex].hold = false;
  } else if (strcmp(tokens[2], "kp") == 0) {
    if (!parseIntToken(tokens[3], intValue) || !inRange(intValue, 0, 10000)) {
      sendErr("BAD_VALUE");
      return;
    }
    cfg.kp = static_cast<uint16_t>(intValue);
  } else if (strcmp(tokens[2], "kd") == 0) {
    if (!parseIntToken(tokens[3], intValue) || !inRange(intValue, 0, 10000)) {
      sendErr("BAD_VALUE");
      return;
    }
    cfg.kd = static_cast<uint16_t>(intValue);
  } else if (strcmp(tokens[2], "minpwm") == 0) {
    if (!parseIntToken(tokens[3], intValue) || !inRange(intValue, 0, 255)) {
      sendErr("BAD_VALUE");
      return;
    }
    cfg.minPwm = clampPwm(intValue, cfg.maxPwm);
  } else if (strcmp(tokens[2], "tolerance") == 0) {
    if (!parseFloatToken(tokens[3], floatValue) || floatValue < 0.1f || floatValue > 20.0f) {
      sendErr("BAD_VALUE");
      return;
    }
    BrakePointConfigValidation validation;
    if (axisIndex < 3 && axisHasAnyBrakePoint(axisIndex) &&
        !brakePointConfigurationMayChange(axisIndex, validation)) {
      sendBrakePointConfigError(
          axisIndex, BP_INVALID_BOUNDARY, validation);
      return;
    }
    const float previousTolerance = cfg.toleranceDeg;
    cfg.toleranceDeg = floatValue;
    if (axisIndex < 3 && axisHasAnyBrakePoint(axisIndex) &&
        !validateBrakePointConfiguration(
            axisIndex,
            settings.brakePointSlots[axisIndex],
            validation)) {
      cfg.toleranceDeg = previousTolerance;
      sendBrakePointConfigError(
          axisIndex, BP_INVALID_BOUNDARY, validation);
      return;
    }
  } else if (strcmp(tokens[2], "bp") == 0 ||
             strcmp(tokens[2], "breakpoint") == 0) {
    if (axisIndex > 2) {
      sendErr("POSITION_ONLY_J1_J3");
      return;
    }
    BrakePointConfigValidation validation;
    if (!brakePointConfigurationMayChange(axisIndex, validation)) {
      sendBrakePointConfigError(
          axisIndex, BP_INVALID_BOUNDARY, validation);
      return;
    }
    int slotOneBased = 0;
    if (count < 5 ||
        !parseIntToken(tokens[3], slotOneBased) ||
        !inRange(slotOneBased, 1, BP_SLOT_COUNT)) {
      validation.error = BrakePointConfigError::BadSlot;
      sendBrakePointConfigError(
          axisIndex, BP_INVALID_BOUNDARY, validation);
      return;
    }
    const uint8_t slot = static_cast<uint8_t>(slotOneBased - 1);
    BrakePointMode modeValue = BrakePointMode::Disabled;
    if (!parseBrakePointMode(tokens[4], modeValue)) {
      validation.error = BrakePointConfigError::BadMode;
      validation.boundary = slot;
      sendBrakePointConfigError(axisIndex, slot, validation);
      return;
    }
    BrakePointSlot candidateSlots[BP_SLOT_COUNT];
    memcpy(candidateSlots,
           settings.brakePointSlots[axisIndex],
           sizeof(candidateSlots));
    candidateSlots[slot].mode = static_cast<uint8_t>(modeValue);
    if (modeValue != BrakePointMode::Disabled) {
      if (count < 6 ||
          !parseFloatToken(tokens[5], floatValue) ||
          !isfinite(floatValue)) {
        validation.error = BrakePointConfigError::BadAngle;
        validation.boundary = slot;
        sendBrakePointConfigError(axisIndex, slot, validation);
        return;
      }
      candidateSlots[slot].angleDeg = normalize360(floatValue);
    } else {
      candidateSlots[slot].angleDeg = 0.0f;
    }
    if (!validateBrakePointConfiguration(
            axisIndex,
            candidateSlots,
            validation)) {
      sendBrakePointConfigError(axisIndex, slot, validation);
      return;
    }
    memcpy(settings.brakePointSlots[axisIndex],
           candidateSlots,
           sizeof(candidateSlots));
    resetBrakePointRuntime(axisIndex);
  } else if (strcmp(tokens[2], "bpreact") == 0 ||
             strcmp(tokens[2], "bpdecel") == 0 ||
             strcmp(tokens[2], "bplead") == 0 ||
             strcmp(tokens[2], "bpclear") == 0 ||
             strcmp(tokens[2], "bprecpwm") == 0 ||
             strcmp(tokens[2], "bprectime") == 0) {
    if (axisIndex > 2) {
      sendErr("POSITION_ONLY_J1_J3");
      return;
    }
    BrakePointConfigValidation validation;
    if (!brakePointConfigurationMayChange(axisIndex, validation)) {
      sendBrakePointConfigError(
          axisIndex, BP_INVALID_BOUNDARY, validation);
      return;
    }
    BrakePointTuning &bp = settings.brakePoint[axisIndex];
    const BrakePointTuning previousTuning = bp;
    if (strcmp(tokens[2], "bpreact") == 0) {
      if (!parseIntToken(tokens[3], intValue) || !inRange(intValue, 0, 500)) {
        sendErr("BAD_VALUE");
        return;
      }
      bp.reactionMs = static_cast<uint16_t>(intValue);
    } else if (strcmp(tokens[2], "bpdecel") == 0) {
      if (!parseIntToken(tokens[3], intValue) || !inRange(intValue, 50, 20000)) {
        sendErr("BAD_VALUE");
        return;
      }
      bp.decelDps2 = static_cast<uint16_t>(intValue);
    } else if (strcmp(tokens[2], "bplead") == 0) {
      if (!parseFloatToken(tokens[3], floatValue) ||
          floatValue < 0.1f || floatValue > 20.0f) {
        sendErr("BAD_VALUE");
        return;
      }
      bp.leadMarginDeg = floatValue;
    } else if (strcmp(tokens[2], "bpclear") == 0) {
      if (!parseFloatToken(tokens[3], floatValue) ||
          floatValue < 0.2f || floatValue > 20.0f) {
        sendErr("BAD_VALUE");
        return;
      }
      bp.recoveryClearanceDeg = floatValue;
    } else if (strcmp(tokens[2], "bprecpwm") == 0) {
      if (!parseIntToken(tokens[3], intValue) || !inRange(intValue, 0, 255)) {
        sendErr("BAD_VALUE");
        return;
      }
      bp.recoveryMaxPwm = static_cast<uint8_t>(intValue);
    } else {
      if (!parseIntToken(tokens[3], intValue) || !inRange(intValue, 250, 10000)) {
        sendErr("BAD_VALUE");
        return;
      }
      bp.recoveryTimeoutMs = static_cast<uint16_t>(intValue);
    }
    if (!validateBrakePointConfiguration(
            axisIndex,
            settings.brakePointSlots[axisIndex],
            validation)) {
      bp = previousTuning;
      sendBrakePointConfigError(
          axisIndex, BP_INVALID_BOUNDARY, validation);
      return;
    }
  } else if (strcmp(tokens[2], "stopmode") == 0) {
    if (strcmp(tokens[3], "coast") == 0) {
      cfg.stopMode = static_cast<uint8_t>(StopMode::Coast);
    } else if (strcmp(tokens[3], "brake") == 0) {
      cfg.stopMode = static_cast<uint8_t>(StopMode::Brake);
    } else if (strcmp(tokens[3], "hold") == 0) {
      cfg.stopMode = static_cast<uint8_t>(StopMode::Hold);
    } else if (strcmp(tokens[3], "hybrid") == 0) {
      cfg.stopMode = static_cast<uint8_t>(StopMode::Hybrid);
    } else {
      sendErr("BAD_STOPMODE");
      return;
    }
  } else {
    sendErr("BAD_PARAM");
    return;
  }

  markSettingsDirty();
  sendOk("UPDATED_RAM");
}

void handleArmDriveCommand(char **tokens, uint8_t count, Direction direction) {
  if (count < 2) {
    sendErr("USAGE_DRIVE");
    return;
  }
  uint8_t axisOneBased = 0;
  if (!parseAxis(tokens[1], axisOneBased)) {
    sendErr("BAD_AXIS");
    return;
  }
  int pwm = -1;
  if (count >= 3 && (!parseIntToken(tokens[2], pwm) || !inRange(pwm, 0, 255))) {
    sendErr("BAD_PWM");
    return;
  }
  driveArmAxis(axisOneBased, direction, pwm);
}

void applyAxisValue(uint8_t axisOneBased, int value) {
  if (mode != SystemMode::Arm && mode != SystemMode::ArmTest) {
    sendErr("MODE_NOT_ARM_OR_ARMTEST");
    return;
  }

  const uint8_t axisIndex = axisOneBased - 1;
  const uint16_t dead = settings.axis[axisIndex].dead;
  if (mode == SystemMode::Arm && axisIndex < 3 &&
      anyBrakePointHasExclusiveControl()) {
    if (abs(value) <= dead) {
      for (uint8_t guardedAxis = 0;
           guardedAxis < 3;
           ++guardedAxis) {
        noteBrakePointNeutral(guardedAxis);
      }
    }
    sendErr("BRAKEPOINT_ARM_LOCKED");
    return;
  }
  if (abs(value) <= dead) {
    noteBrakePointNeutral(axisIndex);
    if (axisIndex < 3 && brakePointHasExclusiveControl(axisIndex)) {
      sendOk("AXIS_STOP");
      return;
    }
    stopAxis(axisOneBased);
    sendOk("AXIS_STOP");
    return;
  }

  const bool positive = value > 0;
  const int magnitude = abs(value);
  const int pwm = (static_cast<int32_t>(magnitude) * settings.axis[axisIndex].maxPwm) / 1000L;
  driveArmAxis(axisOneBased, positive ? Direction::Forward : Direction::Backward, pwm);
}

void handleAxisCommand(char **tokens, uint8_t count) {
  if (count < 3) {
    sendErr("USAGE_AXIS");
    return;
  }

  uint8_t axisOneBased = 0;
  int value = 0;
  if (!parseAxis(tokens[1], axisOneBased) || !parseIntToken(tokens[2], value) || !inRange(value, -1000, 1000)) {
    sendErr("BAD_AXIS_VALUE");
    return;
  }

  applyAxisValue(axisOneBased, value);
}

bool applyKeybindAxis(uint8_t axisIndex) {
  if (axisIndex >= AXIS_COUNT) {
    return false;
  }

  const uint8_t axisOneBased = axisIndex + 1;
  const bool forwardActive = keybindForwardActive[axisIndex];
  const bool backwardActive = keybindBackwardActive[axisIndex];

  if (mode == SystemMode::Arm && axisIndex < 3 &&
      anyBrakePointHasExclusiveControl()) {
    if (!forwardActive && !backwardActive) {
      for (uint8_t guardedAxis = 0;
           guardedAxis < 3;
           ++guardedAxis) {
        noteBrakePointNeutral(guardedAxis);
      }
    }
    return true;
  }

  if (forwardActive == backwardActive) {
    stopAxis(axisOneBased);
    return true;
  }

  const Direction direction = forwardActive ? Direction::Forward : Direction::Backward;
  return driveArmAxisSilent(axisOneBased, direction, -1);
}

void handleJoyButtonCommand(char **tokens, uint8_t count) {
  if (count < 4) {
    sendErr("USAGE_JOY_BUTTON");
    return;
  }

  int buttonId = 0;
  int state = 0;
  if (!parseIntToken(tokens[2], buttonId) || !inRange(buttonId, 0, 32767) ||
      !parseIntToken(tokens[3], state) || !inRange(state, 0, 1)) {
    sendErr("BAD_JOY_BUTTON");
    return;
  }

  uint8_t matchedMask = 0;
  const bool pressed = state == 1;
  const bool servoNextMatched = settings.servo.keybindNext == buttonId;
  const bool servoPreviousMatched =
      settings.servo.keybindPrevious == buttonId;
  for (uint8_t axisIndex = 0; axisIndex < AXIS_COUNT; ++axisIndex) {
    AxisConfig &cfg = settings.axis[axisIndex];
    if (cfg.keybindForward == buttonId) {
      keybindForwardActive[axisIndex] = pressed;
      matchedMask |= static_cast<uint8_t>(1U << axisIndex);
    }
    if (cfg.keybindBackward == buttonId) {
      keybindBackwardActive[axisIndex] = pressed;
      matchedMask |= static_cast<uint8_t>(1U << axisIndex);
    }
  }

  if (mode == SystemMode::Drill &&
      (servoNextMatched || servoPreviousMatched)) {
    if (!pressed) {
      sendOk("JOY_RELEASE");
      return;
    }
    if (!servoRuntime.stateInitialized) {
      servoRuntime.error = ServoError::ReferenceRequired;
      sendErr("SERVO_STATE_NOT_INITIALIZED");
      return;
    }
    if (servoRuntime.referencePulseAssumed) {
      sendErr("SERVO_REFERENCE_PULSE_UNKNOWN");
      return;
    }
    if (servoRuntime.motion != ServoMotion::Idle) {
      sendErr("SERVO_BUSY");
      return;
    }
    if (servoRuntime.currentState < SERVO_STATE_MIN ||
        servoRuntime.currentState > SERVO_STATE_MAX) {
      sendErr("SERVO_CURRENT_STATE_UNKNOWN_USE_DIRECT_TARGET");
      return;
    }
    int targetState = servoRuntime.currentState;
    if (servoNextMatched && !servoPreviousMatched) {
      ++targetState;
    } else if (servoPreviousMatched && !servoNextMatched) {
      --targetState;
    } else {
      sendErr("SERVO_KEYBIND_CONFLICT");
      return;
    }
    if (!inRange(targetState, SERVO_STATE_MIN, SERVO_STATE_MAX)) {
      sendOk("SERVO_LIMIT_IGNORED");
      return;
    }
    if (!startServoStateMove(static_cast<uint8_t>(targetState))) {
      sendErr(servoRuntime.error == ServoError::TargetNotCalibrated
                  ? "SERVO_TARGET_NOT_CALIBRATED"
                  : servoRuntime.safetyReason == ServoSafetyReason::PulseRange
                        ? "SERVO_TARGET_PULSE_RANGE"
                        : "SERVO_MOVE_REJECTED");
      return;
    }
    sendOk("SERVO_TARGET_ACCEPTED");
    return;
  }

  if (matchedMask == 0 &&
      !servoNextMatched && !servoPreviousMatched) {
    sendOk("JOY_IGNORED");
    return;
  }

  if (mode != SystemMode::Arm && mode != SystemMode::ArmTest) {
    sendErr("MODE_NOT_ARM_OR_ARMTEST");
    return;
  }

  for (uint8_t axisIndex = 0; axisIndex < AXIS_COUNT; ++axisIndex) {
    if ((matchedMask & (1U << axisIndex)) != 0) {
      applyKeybindAxis(axisIndex);
    }
  }
  sendOk(pressed ? "JOY_PRESS" : "JOY_RELEASE");
}

void handleJoyCommand(char **tokens, uint8_t count) {
  if (count < 2) {
    sendErr("USAGE_JOY");
    return;
  }

  if (strcmp(tokens[1], "disconnect") == 0 ||
      strcmp(tokens[1], "lost") == 0) {
    stopAll(false);
    mode = SystemMode::Safe;
    servoRuntime.safetyReason = ServoSafetyReason::GamepadDisconnect;
    sendOk("GAMEPAD_DISCONNECT MODE_SAFE");
    return;
  }

  if (strcmp(tokens[1], "axis") == 0) {
    if (count < 4) {
      sendErr("USAGE_JOY_AXIS");
      return;
    }
    int logicalAxis = 0;
    int value = 0;
    if (!parseIntToken(tokens[2], logicalAxis) ||
        !parseIntToken(tokens[3], value) ||
        !inRange(value, -1000, 1000)) {
      sendErr("BAD_JOY_AXIS");
      return;
    }
    uint8_t axisOneBased = 0;
    if (!parseAxis(tokens[2], axisOneBased)) {
      sendErr("BAD_JOY_AXIS");
      return;
    }
    applyAxisValue(axisOneBased, value);
    return;
  }

  if (strcmp(tokens[1], "servo") == 0) {
    if (count < 3) {
      sendErr("USAGE_JOY_SERVO");
      return;
    }
    sendErr("SERVO_GAMEPAD_CONTROL_DISABLED");
    return;
  }

  if (strcmp(tokens[1], "button") == 0 || strcmp(tokens[1], "btn") == 0) {
    handleJoyButtonCommand(tokens, count);
    return;
  }

  sendErr("BAD_JOY_TARGET");
}

void invalidateAxisSensor(uint8_t axisIndex) {
  AxisRuntime &runtime = axisRuntime[axisIndex];
  sensorFaultBits |= static_cast<uint8_t>(1U << axisIndex);
  const bool encoderControlledOutput =
      targets[axisIndex].active ||
      runtime.brakePointPhase != BrakePointPhase::Inactive;
  runtime.angleValid = false;
  runtime.hasPreviousAngle = false;
  runtime.velocityRawDps = 0.0f;
  runtime.velocityDps = 0.0f;
  targets[axisIndex].active = false;
  targets[axisIndex].hold = false;
  resetPositionControllerRuntime(axisIndex);
  resetBrakePointRuntime(axisIndex);
  if (encoderControlledOutput) {
    // Only encoder-dependent output is removed. A direct manual command that
    // was already driving the joint remains untouched by sensor loss.
    driveMotorHardware(axisIndex, Direction::Stop, 0);
  }
}

void handleAxisSensorFailure(uint8_t axisIndex, uint32_t nowUs) {
  AxisRuntime &runtime = axisRuntime[axisIndex];
  ++as5600_error_count;
  if (runtime.consecutiveReadErrors < UINT16_MAX) {
    ++runtime.consecutiveReadErrors;
  }
  runtime.nextReadAllowedUs =
      nowUs + as5600RetryBackoffUs(runtime.consecutiveReadErrors);

  const bool sampleIsStale = runtime.lastSampleUs == 0 ||
                             nowUs - runtime.lastSampleUs > AS5600_STALE_TIMEOUT_US;
  if (sampleIsStale) {
    invalidateAxisSensor(axisIndex);
  }
}

bool sampleAxisSensor(uint8_t axisIndex, uint32_t nowUs) {
  if (axisIndex >= 3) {
    return false;
  }

  AxisRuntime &runtime = axisRuntime[axisIndex];
  if (runtime.nextReadAllowedUs != 0 &&
      static_cast<int32_t>(nowUs - runtime.nextReadAllowedUs) < 0) {
    return false;
  }
  runtime.lastReadAttemptUs = nowUs;

  uint16_t rawAngle = 0;
  float sensorAngleDeg = 0.0f;
  if (!sensors.readAngle(axisIndex, rawAngle, sensorAngleDeg)) {
    handleAxisSensorFailure(axisIndex, nowUs);
    return false;
  }

  const float correctedAngleDeg =
      normalize360(sensorAngleDeg - settings.axis[axisIndex].zeroOffsetDeg);
  const bool hadValidSample = runtime.angleValid && runtime.lastSampleUs != 0;
  const float previousAngleDeg = runtime.angleDeg;
  const uint32_t dtUs = nowUs - runtime.lastSampleUs;
  float deltaAngleDeg = 0.0f;

  if (hadValidSample) {
    deltaAngleDeg = shortestErrorDeg(correctedAngleDeg, runtime.angleDeg);
    if (dtUs > 0 && dtUs <= AS5600_MAX_VELOCITY_DT_US) {
      const float dtSeconds = static_cast<float>(dtUs) * 1.0e-6f;
      const float rawVelocityDps = deltaAngleDeg / dtSeconds;
      if (fabsf(rawVelocityDps) > AS5600_MAX_VALID_VELOCITY_DPS) {
        handleAxisSensorFailure(axisIndex, nowUs);
        return false;
      }

      const float rcSeconds = 1.0f / (6.28318530718f * AS5600_VELOCITY_CUTOFF_HZ);
      const float alpha = dtSeconds / (rcSeconds + dtSeconds);
      runtime.velocityRawDps = rawVelocityDps;
      runtime.velocityDps += alpha * (rawVelocityDps - runtime.velocityDps);
      recordAS5600SampleTiming(dtUs);
    } else {
      runtime.velocityRawDps = 0.0f;
      runtime.velocityDps = 0.0f;
    }
    runtime.multiTurnAngleDeg += deltaAngleDeg;
  } else {
    runtime.multiTurnAngleDeg = correctedAngleDeg;
    runtime.velocityRawDps = 0.0f;
    runtime.velocityDps = 0.0f;
    resetPositionControllerRuntime(axisIndex);
  }

  runtime.rawAngle = rawAngle;
  runtime.sensorAngleDeg = sensorAngleDeg;
  runtime.previousAngleDeg = hadValidSample ? previousAngleDeg : correctedAngleDeg;
  runtime.hasPreviousAngle = hadValidSample;
  runtime.angleDeg = correctedAngleDeg;
  runtime.lastSampleUs = nowUs;
  runtime.consecutiveReadErrors = 0;
  runtime.nextReadAllowedUs = 0;
  runtime.angleValid = true;
  sensorFaultBits &= static_cast<uint8_t>(~(1U << axisIndex));
  updateBrakePointSensorState(axisIndex, previousAngleDeg, correctedAngleDeg, hadValidSample);

  ++as5600_update_count;
  as5600_last_axis = axisIndex;
  as5600_velocity_raw_dps = runtime.velocityRawDps;
  as5600_velocity_filtered_dps = runtime.velocityDps;
  return true;
}

bool getFreshAxisAngle(uint8_t axisIndex, float &angleDeg) {
  if (axisIndex >= 3) {
    return false;
  }
  const AxisRuntime &runtime = axisRuntime[axisIndex];
  const uint32_t nowUs = GetMicroseconds();
  if ((sensorFaultBits & (1U << axisIndex)) != 0 ||
      !runtime.angleValid || runtime.lastSampleUs == 0 ||
      nowUs - runtime.lastSampleUs > AS5600_STALE_TIMEOUT_US) {
    return false;
  }
  angleDeg = runtime.angleDeg;
  return true;
}

void AS5600_UpdateFast() {
  const uint8_t axisIndex = nextAS5600Axis;
  nextAS5600Axis = static_cast<uint8_t>((nextAS5600Axis + 1U) % 3U);

  if (configuredAS5600Channel(axisIndex) < 0) {
    axisRuntime[axisIndex].angleValid = false;
    axisRuntime[axisIndex].consecutiveReadErrors = 0;
    axisRuntime[axisIndex].nextReadAllowedUs = 0;
    sensorFaultBits &= static_cast<uint8_t>(~(1U << axisIndex));
    return;
  }
  sampleAxisSensor(axisIndex, GetMicroseconds());
}

void serviceSensorBusRecovery() {
  const bool recovered = sensors.serviceRecovery();
  i2c_error_count = sensors.totalErrorCount();
  i2c_recovery_count = sensors.recoveryCount();
  if (!recovered) {
    return;
  }

  // A peripheral restart invalidates the continuity assumptions used by
  // position/brake-point control. Manual motor commands keep their existing
  // behavior, but every encoder-dependent output is cancelled immediately.
  for (uint8_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
    axisRuntime[axisIndex].nextReadAllowedUs = 0;
    if (configuredAS5600Channel(axisIndex) >= 0) {
      invalidateAxisSensor(axisIndex);
    }
  }
}

void handleZeroCommand(char **tokens, uint8_t count) {
  if (count < 2) {
    sendErr("USAGE_ZERO");
    return;
  }
  uint8_t axisOneBased = 0;
  if (!parseAxis(tokens[1], axisOneBased) || axisOneBased > 3) {
    sendErr("POSITION_ONLY_J1_J3");
    return;
  }

  const uint8_t axisIndex = axisOneBased - 1;
  if (axisHasAnyBrakePoint(axisIndex)) {
    if (currentAckId != nullptr) {
      LinkSerial.print("ACK ");
      LinkSerial.print(currentAckId);
      LinkSerial.print(' ');
    }
    LinkSerial.print("ERR ZERO J=");
    LinkSerial.print(axisOneBased);
    LinkSerial.println(" REASON=BP_ENABLED");
    return;
  }
  float current = 0.0f;
  if (!getFreshAxisAngle(axisIndex, current)) {
    sendErr("SENSOR_NOT_READY");
    return;
  }

  AxisRuntime &runtime = axisRuntime[axisIndex];
  settings.axis[axisIndex].zeroOffsetDeg = runtime.sensorAngleDeg;
  runtime.angleDeg = 0.0f;
  runtime.multiTurnAngleDeg = 0.0f;
  runtime.velocityRawDps = 0.0f;
  runtime.velocityDps = 0.0f;
  runtime.previousAngleDeg = 0.0f;
  runtime.hasPreviousAngle = false;
  resetBrakePointRuntime(axisIndex);
  markSettingsDirty();
  sendOk("ZERO_UPDATED_RAM");
}

void handleGotoCommand(char **tokens, uint8_t count, bool relative) {
  if (count < 3) {
    sendErr("USAGE_GOTO");
    return;
  }
  if (mode == SystemMode::ArmTest) {
    sendErr("POSITION_DISABLED_ARMTEST");
    return;
  }
  if (mode != SystemMode::Arm) {
    sendErr("MODE_NOT_ARM");
    return;
  }

  uint8_t axisOneBased = 0;
  float angle = 0.0f;
  if (!parseAxis(tokens[1], axisOneBased) || axisOneBased > 3 || !parseFloatToken(tokens[2], angle)) {
    sendErr("BAD_POSITION_CMD");
    return;
  }

  float current = 0.0f;
  const uint8_t axisIndex = axisOneBased - 1;
  if (anyBrakePointHasExclusiveControl()) {
    sendErr("BRAKEPOINT_ARM_LOCKED");
    return;
  }
  if (!getFreshAxisAngle(axisIndex, current)) {
    sendErr("SENSOR_NOT_READY");
    return;
  }

  const float targetDeg =
      relative ? normalize360(current + angle) : normalize360(angle);
  uint8_t blockingBoundary = BP_INVALID_BOUNDARY;
  if (!brakePointPathAllowed(
          axisIndex, current, targetDeg, blockingBoundary)) {
    if (currentAckId != nullptr) {
      LinkSerial.print("ACK ");
      LinkSerial.print(currentAckId);
      LinkSerial.print(' ');
    }
    LinkSerial.print("ERR TARGET_BLOCKED_BP J=");
    LinkSerial.print(axisOneBased);
    LinkSerial.print(" BP=");
    printBrakePointBoundaryId(blockingBoundary);
    LinkSerial.println();
    return;
  }
  targets[axisIndex].targetDeg = targetDeg;
  targets[axisIndex].active = true;
  targets[axisIndex].hold = false;
  resetPositionControllerRuntime(axisIndex);
  sendOk("TARGET_SET");
}

template <typename ConfigType>
void handleDrillConfigCommand(const char *name,
                              char **tokens,
                              uint8_t count,
                              ConfigType &cfg) {
  if (count < 2) {
    sendErr("USAGE_DRILL_CONFIG");
    return;
  }

  int value = 0;
  if (strcmp(tokens[1], "default") == 0) {
    if (count < 3 || !parseIntToken(tokens[2], value) || !inRange(value, 0, 255)) {
      sendErr("BAD_VALUE");
      return;
    }
    cfg.defaultPwm = clampPwm(value, cfg.maxPwm);
    markSettingsDirty();
    sendOk("UPDATED_RAM");
    return;
  }

  if (strcmp(tokens[1], "invert") == 0) {
    if (count < 3 || !parseIntToken(tokens[2], value) || !inRange(value, 0, 1)) {
      sendErr("BAD_VALUE");
      return;
    }
    cfg.invert = static_cast<uint8_t>(value);
    markSettingsDirty();
    sendOk("UPDATED_RAM");
    return;
  }

  if (strcmp(tokens[1], "maxpwm") == 0) {
    if (count < 3 || !parseIntToken(tokens[2], value) || !inRange(value, 0, 255)) {
      sendErr("BAD_VALUE");
      return;
    }
    cfg.maxPwm = static_cast<uint8_t>(value);
    if (cfg.defaultPwm > cfg.maxPwm) {
      cfg.defaultPwm = cfg.maxPwm;
    }
    markSettingsDirty();
    sendOk("UPDATED_RAM");
    return;
  }

  LinkSerial.print("ERR BAD_");
  LinkSerial.println(name);
}

void handleElevatorCommand(char **tokens, uint8_t count) {
  if (mode != SystemMode::Drill) {
    sendErr("MODE_NOT_DRILL");
    return;
  }
  if (count < 2) {
    sendErr("USAGE_ELV");
    return;
  }

  int pwm = -1;
  if (count >= 3 && (!parseIntToken(tokens[2], pwm) || !inRange(pwm, 0, 255))) {
    sendErr("BAD_PWM");
    return;
  }

  if (strcmp(tokens[1], "up") == 0) {
    driveElevatorMotor(3, Direction::Forward, pwm);
    driveElevatorMotor(4, Direction::Forward, pwm);
    sendOk("ELV_UP");
  } else if (strcmp(tokens[1], "down") == 0) {
    driveElevatorMotor(3, Direction::Backward, pwm);
    driveElevatorMotor(4, Direction::Backward, pwm);
    sendOk("ELV_DOWN");
  } else if (strcmp(tokens[1], "stop") == 0) {
    stopDrillGroup(true);
    sendOk("ELV_STOP");
  } else {
    handleDrillConfigCommand("ELV", tokens, count, settings.elevator);
  }
}

void handleDrillCommand(char **tokens, uint8_t count) {
  if (mode != SystemMode::Drill) {
    sendErr("MODE_NOT_DRILL");
    return;
  }
  if (count < 2) {
    sendErr("USAGE_DRILL");
    return;
  }

  int pwm = -1;
  if (count >= 3 && (!parseIntToken(tokens[2], pwm) || !inRange(pwm, 0, 255))) {
    sendErr("BAD_PWM");
    return;
  }

  if (strcmp(tokens[1], "dig") == 0) {
    driveDrillMotor(5, Direction::Forward, pwm, settings.drill);
    sendOk("DRILL_DIG");
  } else if (strcmp(tokens[1], "extract") == 0) {
    driveDrillMotor(5, Direction::Backward, pwm, settings.drill);
    sendOk("DRILL_EXTRACT");
  } else if (strcmp(tokens[1], "stop") == 0) {
    stopDrillGroup(false);
    sendOk("DRILL_STOP");
  } else {
    handleDrillConfigCommand("DRILL", tokens, count, settings.drill);
  }
}

void handleServoCommand(char **tokens, uint8_t count) {
  if (mode != SystemMode::Drill) {
    sendErr("MODE_NOT_DRILL");
    return;
  }
  if (count >= 2 && strcmp(tokens[1], "diag") == 0) {
    printServoTimerDiagnostics();
    return;
  }
  if (count >= 3 && strcmp(tokens[1], "pulse") == 0) {
    if (strcmp(tokens[2], "off") == 0) {
      disableServoOutput();
      servoRuntime.motion = ServoMotion::Idle;
      servoRuntime.stateInitialized = false;
      servoRuntime.currentState = 0;
      servoRuntime.targetState = 0;
      servoRuntime.referenceState = 0;
      servoRuntime.pulseReferenceKnown = false;
      servoRuntime.safetyReason = ServoSafetyReason::Stop;
      servoRuntime.error = ServoError::None;
      sendOk("SERVO_PWM_OFF");
      return;
    }
    int pulseUs = 0;
    if (!parseIntToken(tokens[2], pulseUs) ||
        !inRange(pulseUs, SERVO_MIN_US, SERVO_MAX_US)) {
      sendErr("BAD_SERVO_PULSE_800_2200");
      return;
    }
    if (servoRuntime.motion != ServoMotion::Idle) {
      sendErr("SERVO_BUSY");
      return;
    }
    if (!startServoFixedMove(static_cast<uint16_t>(pulseUs))) {
      sendErr("SERVO_TIMER_CONFIGURATION");
      return;
    }
    LinkSerial.print("OK SERVO_FIXED_TARGET_ACCEPTED_US=");
    LinkSerial.println(pulseUs);
    return;
  }
  if (count >= 2 &&
      (strcmp(tokens[1], "forward") == 0 ||
       strcmp(tokens[1], "backward") == 0)) {
    if (servoRuntime.motion != ServoMotion::Idle) {
      sendErr("SERVO_BUSY");
      return;
    }
    int stepUs = DEFAULT_SERVO_MANUAL_STEP_US;
    if (count >= 3 &&
        (!parseIntToken(tokens[2], stepUs) ||
         !inRange(stepUs, 1, MAX_SERVO_MANUAL_STEP_US))) {
      sendErr("BAD_SERVO_STEP_1_100");
      return;
    }
    const bool forward = strcmp(tokens[1], "forward") == 0;
    const int logicalDirection = forward ? 1 : -1;
    const int physicalDirection =
        settings.servo.invert ? -logicalDirection : logicalDirection;
    const int targetPulse =
        static_cast<int>(servoRuntime.currentPulseUs) +
        physicalDirection * stepUs;
    if (!inRange(targetPulse, SERVO_MIN_US, SERVO_MAX_US)) {
      sendErr("SERVO_STEP_RANGE");
      return;
    }
    if (!startServoFixedMove(static_cast<uint16_t>(targetPulse))) {
      sendErr("SERVO_TIMER_CONFIGURATION");
      return;
    }
    LinkSerial.print("OK SERVO_STEP_TARGET_ACCEPTED_US=");
    LinkSerial.println(targetPulse);
    return;
  }
  if (count >= 2 &&
      (strcmp(tokens[1], "manual") == 0 ||
       strcmp(tokens[1], "jog") == 0)) {
    sendErr("SERVO_MANUAL_REMOVED_USE_SERVO_PULSE");
    return;
  }
  if (count < 2) {
    sendErr("USAGE_SERVO_STATE_1_5");
    return;
  }
  int targetState = 0;
  if (!parseIntToken(tokens[1], targetState) ||
      !inRange(targetState, SERVO_STATE_MIN, SERVO_STATE_MAX)) {
    sendErr("BAD_SERVO_STATE");
    return;
  }
  if (!servoRuntime.stateInitialized) {
    servoRuntime.error = ServoError::ReferenceRequired;
    sendErr("SERVO_STATE_NOT_INITIALIZED");
    return;
  }
  if (servoRuntime.referencePulseAssumed) {
    sendErr("SERVO_REFERENCE_PULSE_UNKNOWN");
    return;
  }
  if (servoRuntime.motion != ServoMotion::Idle) {
    sendErr("SERVO_BUSY");
    return;
  }
  if (!startServoStateMove(static_cast<uint8_t>(targetState))) {
    sendErr(servoRuntime.error == ServoError::TargetNotCalibrated
                ? "SERVO_TARGET_NOT_CALIBRATED"
                : servoRuntime.safetyReason == ServoSafetyReason::PulseRange
                      ? "SERVO_TARGET_PULSE_RANGE"
                      : "SERVO_MOVE_REJECTED");
    return;
  }
  sendOk("SERVO_TARGET_ACCEPTED");
}

void handleServoWiggleCommand() {
  if (mode != SystemMode::Drill) {
    sendErr("MODE_NOT_DRILL");
    return;
  }
  if (!servoRuntime.stateInitialized) {
    servoRuntime.error = ServoError::ReferenceRequired;
    sendErr("SERVO_STATE_NOT_INITIALIZED");
    return;
  }
  if (servoRuntime.referencePulseAssumed) {
    sendErr("SERVO_REFERENCE_PULSE_UNKNOWN");
    return;
  }
  if (servoRuntime.currentState < SERVO_STATE_MIN ||
      servoRuntime.currentState > SERVO_STATE_MAX) {
    sendErr("SERVO_CURRENT_STATE_UNKNOWN_MOVE_TO_TUBE_FIRST");
    return;
  }
  if (!startServoWiggle()) {
    sendErr("SERVO_BUSY");
    return;
  }
  sendOk("SERVO_WIGGLE_STARTED");
}

void handleDrillLedCommand(char **tokens, uint8_t count) {
  if (mode != SystemMode::Drill) {
    sendErr("MODE_NOT_DRILL");
    return;
  }
  int ledOneBased = 0;
  int state = 0;
  if (count < 3) {
    sendErr("USAGE_DRILLLED_1_4_0_1");
    return;
  }
  if (!parseIntToken(tokens[1], ledOneBased) ||
      !inRange(ledOneBased, 1, DRILL_LED_COUNT)) {
    sendErr("BAD_DRILL_LED_NUMBER");
    return;
  }
  if (!parseIntToken(tokens[2], state) || !inRange(state, 0, 1)) {
    sendErr("BAD_DRILL_LED_STATE");
    return;
  }
  setDrillLed(static_cast<uint8_t>(ledOneBased - 1), state == 1);
  sendOk(state == 1 ? "DRILL_LED_ON" : "DRILL_LED_OFF");
}

void triggerFailsafe(ServoSafetyReason reason, const char *acknowledgement) {
  pendingConfirm = ConfirmAction::None;
  stopAll(false);
  mode = SystemMode::Safe;
  servoRuntime.safetyReason = reason;
  sendOk(acknowledgement);
}

void handleFailsafeCommand(char **tokens, uint8_t count) {
  if (count < 2) {
    sendErr("USAGE_FAILSAFE_LINKLOSS_GAMEPAD_ESTOP");
    return;
  }
  if (strcmp(tokens[1], "linkloss") == 0 ||
      strcmp(tokens[1], "timeout") == 0) {
    triggerFailsafe(ServoSafetyReason::LinkLoss,
                    "FAILSAFE_LINK_LOSS MODE_SAFE");
  } else if (strcmp(tokens[1], "gamepad") == 0) {
    triggerFailsafe(ServoSafetyReason::GamepadDisconnect,
                    "FAILSAFE_GAMEPAD MODE_SAFE");
  } else if (strcmp(tokens[1], "estop") == 0) {
    triggerFailsafe(ServoSafetyReason::EmergencyStop,
                    "FAILSAFE_ESTOP MODE_SAFE");
  } else {
    sendErr("BAD_FAILSAFE_REASON");
  }
}

void applyConfirmation(bool accepted) {
  const ConfirmAction action = pendingConfirm;
  pendingConfirm = ConfirmAction::None;
  if (!accepted) {
    sendOk("CANCELLED");
    return;
  }

  if (action == ConfirmAction::ModeArm) {
    enterMode(SystemMode::Arm);
  } else if (action == ConfirmAction::ModeDrill) {
    enterMode(SystemMode::Drill);
  } else if (action == ConfirmAction::ModeArmTest) {
    enterMode(SystemMode::ArmTest);
  } else if (action == ConfirmAction::FactoryReset) {
    stopAll(false);
    disableServoOutput();
    servoRuntime = ServoRuntime{};
    servoRuntime.safetyReason = ServoSafetyReason::FactoryReset;
    setDefaults();
    markSettingsDirty();
    mode = SystemMode::Safe;
    sendOk("FACTORY_RESET_RAM MODE SAFE");
  }
}

void tokenizeAndLower(char *line, char **tokens, uint8_t &count) {
  for (char *p = line; *p != '\0'; ++p) {
    if (*p >= 'A' && *p <= 'Z') {
      *p = static_cast<char>(*p - 'A' + 'a');
    }
  }

  count = 0;
  char *token = strtok(line, " \t,");
  while (token != nullptr && count < 8) {
    tokens[count++] = token;
    token = strtok(nullptr, " \t,");
  }
}

bool handlePendingSafetyCommand(char **tokens, uint8_t count) {
  if (count == 0) {
    return false;
  }

  if (strcmp(tokens[0], "mode") == 0 && count >= 2 &&
      (strcmp(tokens[1], "safe") == 0 || strcmp(tokens[1], "none") == 0)) {
    pendingConfirm = ConfirmAction::None;
    enterMode(SystemMode::Safe);
    return true;
  }

  if (strcmp(tokens[0], "stop") == 0) {
    pendingConfirm = ConfirmAction::None;
    if (count == 1) {
      stopAll(true);
      sendOk("STOP_ALL");
    } else {
      uint8_t axisOneBased = 0;
      if (!parseAxis(tokens[1], axisOneBased)) {
        sendErr("BAD_AXIS");
      } else {
        if (axisOneBased <= 3) {
          resetBrakePointRuntime(axisOneBased - 1);
        }
        stopAxis(axisOneBased);
        sendOk("STOP_AXIS");
      }
    }
    return true;
  }

  if (strcmp(tokens[0], "stopall") == 0 || strcmp(tokens[0], "allstop") == 0) {
    pendingConfirm = ConfirmAction::None;
    stopAll(false);
    sendOk("STOP_ALL");
    return true;
  }

  if (strcmp(tokens[0], "estop") == 0) {
    triggerFailsafe(ServoSafetyReason::EmergencyStop,
                    "FAILSAFE_ESTOP MODE_SAFE");
    return true;
  }

  if (strcmp(tokens[0], "failsafe") == 0) {
    handleFailsafeCommand(tokens, count);
    return true;
  }

  if (strcmp(tokens[0], "quit") == 0 || strcmp(tokens[0], "exit") == 0) {
    pendingConfirm = ConfirmAction::None;
    stopAll(false);
    mode = SystemMode::Safe;
    sendOk("BYE MODE SAFE");
    return true;
  }

  return false;
}

void handleLine(char *line) {
  currentAckId = nullptr;
  char *tokens[8] = {};
  uint8_t count = 0;
  tokenizeAndLower(line, tokens, count);
  if (count == 0) {
    return;
  }

  if (strcmp(tokens[0], "seq") == 0) {
    if (count < 3) {
      if (count >= 2) {
        currentAckId = tokens[1];
      }
      sendErr("BAD_SEQ");
      return;
    }
    currentAckId = tokens[1];
    for (uint8_t i = 0; i < count - 2; ++i) {
      tokens[i] = tokens[i + 2];
    }
    count = static_cast<uint8_t>(count - 2);
  }

  // Silent compatibility command used by existing H7/Test GUI integrations.
  // F401 no longer applies a UART inactivity timeout.
  if (strcmp(tokens[0], "hb") == 0 ||
      strcmp(tokens[0], "heartbeat") == 0) {
    return;
  }

  if (pendingConfirm != ConfirmAction::None) {
    if (strcmp(tokens[0], "y") == 0 || strcmp(tokens[0], "yes") == 0) {
      applyConfirmation(true);
      return;
    }
    if (strcmp(tokens[0], "n") == 0 || strcmp(tokens[0], "no") == 0) {
      applyConfirmation(false);
      return;
    }
    if (handlePendingSafetyCommand(tokens, count)) {
      return;
    }
    sendErr("CONFIRM_REQUIRED_Y_OR_N");
    return;
  }

  if (strcmp(tokens[0], "help") == 0) {
    printHelp();
  } else if (strcmp(tokens[0], "params") == 0) {
    printParams();
  } else if (strcmp(tokens[0], "get") == 0) {
    handleGetCommand(tokens, count);
  } else if (strcmp(tokens[0], "stream") == 0) {
    handleStreamCommand(tokens, count);
  } else if (strcmp(tokens[0], "save") == 0) {
    handleSaveCommand();
  } else if (strcmp(tokens[0], "mode") == 0) {
    handleModeCommand(tokens, count);
  } else if (strcmp(tokens[0], "forward") == 0) {
    handleArmDriveCommand(tokens, count, Direction::Forward);
  } else if (strcmp(tokens[0], "backward") == 0) {
    handleArmDriveCommand(tokens, count, Direction::Backward);
  } else if (strcmp(tokens[0], "axis") == 0) {
    handleAxisCommand(tokens, count);
  } else if (strcmp(tokens[0], "joy") == 0) {
    handleJoyCommand(tokens, count);
  } else if (strcmp(tokens[0], "stop") == 0) {
    if (count == 1) {
      stopAll(true);
      sendOk("STOP_ALL");
    } else {
      uint8_t axisOneBased = 0;
      if (!parseAxis(tokens[1], axisOneBased)) {
        sendErr("BAD_AXIS");
      } else {
        if (axisOneBased <= 3) {
          resetBrakePointRuntime(axisOneBased - 1);
        }
        stopAxis(axisOneBased);
        sendOk("STOP_AXIS");
      }
    }
  } else if (strcmp(tokens[0], "stopall") == 0 || (strcmp(tokens[0], "allstop") == 0)) {
    stopAll(false);
    sendOk("STOP_ALL");
  } else if (strcmp(tokens[0], "zero") == 0) {
    handleZeroCommand(tokens, count);
  } else if (strcmp(tokens[0], "goto") == 0) {
    handleGotoCommand(tokens, count, false);
  } else if (strcmp(tokens[0], "rotate") == 0 || strcmp(tokens[0], "move") == 0) {
    handleGotoCommand(tokens, count, true);
  } else if (strcmp(tokens[0], "set") == 0) {
    handleSetCommand(tokens, count);
  } else if (strcmp(tokens[0], "elv") == 0) {
    handleElevatorCommand(tokens, count);
  } else if (strcmp(tokens[0], "drill") == 0) {
    handleDrillCommand(tokens, count);
  } else if (strcmp(tokens[0], "servo") == 0) {
    handleServoCommand(tokens, count);
  } else if (strcmp(tokens[0], "servowiggle") == 0) {
    handleServoWiggleCommand();
  } else if (strcmp(tokens[0], "drillled") == 0) {
    handleDrillLedCommand(tokens, count);
  } else if (strcmp(tokens[0], "failsafe") == 0) {
    handleFailsafeCommand(tokens, count);
  } else if (strcmp(tokens[0], "estop") == 0) {
    triggerFailsafe(ServoSafetyReason::EmergencyStop,
                    "FAILSAFE_ESTOP MODE_SAFE");
  } else if (strcmp(tokens[0], "factoryreset") == 0) {
    if (count >= 2 && strcmp(tokens[1], "confirm") == 0) {
      pendingConfirm = ConfirmAction::FactoryReset;
      applyConfirmation(true);
    } else {
      pendingConfirm = ConfirmAction::FactoryReset;
      sendLine("CONFIRM FACTORY RESET? Y/N");
    }
  } else if (strcmp(tokens[0], "quit") == 0 || strcmp(tokens[0], "exit") == 0) {
    stopAll(false);
    mode = SystemMode::Safe;
    servoRuntime.safetyReason = ServoSafetyReason::SafeMode;
    sendOk("BYE MODE SAFE");
  } else {
    sendErr("UNKNOWN_CMD");
  }
}

void serviceSerial() {
  uint16_t servicedBytes = 0;
  while (LinkSerial.available() > 0 &&
         servicedBytes < SERIAL_SERVICE_BYTE_BUDGET) {
    const char c = static_cast<char>(LinkSerial.read());
    ++servicedBytes;
    if (c == '\r' || c == '\n') {
      if (rxLen > 0) {
        rxLine[rxLen] = '\0';
        handleLine(rxLine);
        currentAckId = nullptr;
        rxLen = 0;
      }
    } else if (rxLen < sizeof(rxLine) - 1) {
      rxLine[rxLen++] = c;
    } else {
      rxLen = 0;
      sendErr("LINE_TOO_LONG");
    }
  }
}

void serviceBrakeTimers() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (motors[i].braking && static_cast<int32_t>(now - motors[i].brakeUntilMs) >= 0) {
      coastMotor(i);
    }
  }
}

const char *brakePointApproachName(BrakePointApproach approach) {
  switch (approach) {
    case BrakePointApproach::FromBelow: return "FROM_BELOW";
    case BrakePointApproach::FromAbove: return "FROM_ABOVE";
    default: return "INACTIVE";
  }
}

const char *brakePointPhaseName(BrakePointPhase phase) {
  switch (phase) {
    case BrakePointPhase::Braking: return "BRAKING";
    case BrakePointPhase::Settling: return "SETTLING";
    case BrakePointPhase::Recovering: return "RECOVERING";
    case BrakePointPhase::Guarded: return "GUARDED";
    case BrakePointPhase::Fault: return "FAULT";
    default: return "INACTIVE";
  }
}

void debugBrakePoint(uint8_t axisIndex,
                     Direction logicalDirection,
                     uint8_t requestedPwm,
                     uint8_t filteredPwm,
                     bool force = false) {
#if BREAKPOINT_DEBUG
  if (axisIndex >= 3) {
    return;
  }
  AxisRuntime &runtime = axisRuntime[axisIndex];
  const uint32_t now = millis();
  if (!force && now - runtime.brakePointLastDebugMs < 50) {
    return;
  }
  runtime.brakePointLastDebugMs = now;
  LinkSerial.print("BPDBG J=");
  LinkSerial.print(axisIndex + 1);
  LinkSerial.print(" ANGLE=");
  LinkSerial.print(runtime.angleDeg, 2);
  LinkSerial.print(" PREV=");
  LinkSerial.print(runtime.previousAngleDeg, 2);
  LinkSerial.print(" REQ_DIR=");
  LinkSerial.print(directionName(logicalDirection));
  LinkSerial.print(" REQ_PWM=");
  LinkSerial.print(requestedPwm);
  LinkSerial.print(" FILTERED_PWM=");
  LinkSerial.print(filteredPwm);
  BrakePointBoundaryView activeBoundary;
  const bool hasActiveBoundary = getBrakePointBoundary(
      axisIndex, runtime.activeBrakePointBoundary, activeBoundary);
  LinkSerial.print(" BP_ID=");
  if (hasActiveBoundary) {
    printBrakePointBoundaryId(runtime.activeBrakePointBoundary);
  } else {
    LinkSerial.print("NONE");
  }
  LinkSerial.print(" BP_MODE=");
  LinkSerial.print(hasActiveBoundary
                       ? brakePointModeName(activeBoundary.mode)
                       : "OFF");
  LinkSerial.print(" BP=");
  if (hasActiveBoundary) {
    LinkSerial.print(activeBoundary.angleDeg, 2);
  } else {
    LinkSerial.print("NA");
  }
  LinkSerial.print(" SIDE=");
  LinkSerial.print(brakePointApproachName(runtime.brakePointApproach));
  LinkSerial.print(" BLOCKED_DIR=");
  if (!hasActiveBoundary) {
    LinkSerial.print("NONE");
  } else if (activeBoundary.mode == BrakePointMode::BlockIncrease ||
             (activeBoundary.mode == BrakePointMode::Wall &&
              runtime.brakePointApproach ==
                  BrakePointApproach::FromBelow)) {
    LinkSerial.print("INCREASE");
  } else if (activeBoundary.mode == BrakePointMode::BlockDecrease ||
             (activeBoundary.mode == BrakePointMode::Wall &&
              runtime.brakePointApproach ==
                  BrakePointApproach::FromAbove)) {
    LinkSerial.print("DECREASE");
  } else {
    LinkSerial.print("NONE");
  }
  LinkSerial.print(" PHASE=");
  LinkSerial.print(brakePointPhaseName(runtime.brakePointPhase));
  LinkSerial.print(" BRAKE=");
  LinkSerial.println(motors[axisIndex].braking ? 1 : 0);
#else
  (void)axisIndex;
  (void)logicalDirection;
  (void)requestedPwm;
  (void)filteredPwm;
  (void)force;
#endif
}

float brakeLeadDistanceDeg(uint8_t axisIndex) {
  const BrakePointTuning &tuning = settings.brakePoint[axisIndex];
  return bpPredictedLeadDistanceDeg(axisRuntime[axisIndex].velocityDps,
                                    tuning.reactionMs,
                                    tuning.decelDps2,
                                    tuning.leadMarginDeg,
                                    BRAKE_MIN_LEAD_DEG,
                                    BRAKE_MAX_LEAD_DEG);
}

float brakePointEffectiveClearanceDeg(uint8_t axisIndex) {
  return configuredBrakePointClearanceDeg(axisIndex);
}

float brakePointRecoveryToleranceDeg(uint8_t axisIndex) {
  float tolerance = settings.axis[axisIndex].toleranceDeg;
  if (tolerance < BP_MIN_RECOVERY_TOLERANCE_DEG) {
    tolerance = BP_MIN_RECOVERY_TOLERANCE_DEG;
  }
  if (tolerance > 0.5f) {
    tolerance = 0.5f;
  }
  return tolerance;
}

bool brakePointDirectionForbidden(BrakePointMode modeValue,
                                  BrakePointApproach approach,
                                  Direction logicalDirection) {
  return bpModeDirectionForbidden(
      modeValue,
      approach,
      static_cast<int8_t>(logicalDirection));
}

BrakePointApproach brakePointApproachFromError(float signedError,
                                                Direction fallbackDirection) {
  if (signedError > 0.0f) {
    return BrakePointApproach::FromBelow;
  }
  if (signedError < 0.0f) {
    return BrakePointApproach::FromAbove;
  }
  return fallbackDirection == Direction::Backward
             ? BrakePointApproach::FromAbove
             : BrakePointApproach::FromBelow;
}

bool brakePointSegmentCrossed(float previousAngleDeg,
                              float currentAngleDeg,
                              float brakePointDeg,
                              BrakePointApproach &approach) {
  return bpSegmentCrossed(previousAngleDeg,
                          currentAngleDeg,
                          brakePointDeg,
                          approach);
}

bool brakePointOnOriginalSide(uint8_t axisIndex,
                              float currentAngleDeg,
                              float &distanceDeg) {
  const AxisRuntime &runtime = axisRuntime[axisIndex];
  BrakePointBoundaryView activeBoundary;
  if (!getBrakePointBoundary(
          axisIndex,
          runtime.activeBrakePointBoundary,
          activeBoundary)) {
    distanceDeg = 0.0f;
    return false;
  }
  return bpOriginalSideDistance(runtime.brakePointApproach,
                                activeBoundary.angleDeg,
                                currentAngleDeg,
                                distanceDeg);
}

bool brakePointBoundaryThreatens(uint8_t axisIndex,
                                 uint8_t boundaryIndex,
                                 float currentAngleDeg,
                                 Direction logicalDirection,
                                 float leadDistanceDeg,
                                 BrakePointApproach &approach,
                                 float &distanceDeg) {
  BrakePointBoundaryView boundary;
  if (!getBrakePointBoundary(axisIndex, boundaryIndex, boundary) ||
      logicalDirection == Direction::Stop) {
    return false;
  }
  approach = boundary.mode == BrakePointMode::Wall
                 ? (logicalDirection == Direction::Forward
                        ? BrakePointApproach::FromBelow
                        : BrakePointApproach::FromAbove)
                 : bpFixedApproachForMode(boundary.mode);
  if (!brakePointDirectionForbidden(
          boundary.mode, approach, logicalDirection)) {
    return false;
  }
  distanceDeg = bpDirectionalDistanceDeg(
      currentAngleDeg,
      boundary.angleDeg,
      static_cast<int8_t>(logicalDirection));
  return distanceDeg <= leadDistanceDeg;
}

bool findThreateningBrakePoint(uint8_t axisIndex,
                               float currentAngleDeg,
                               Direction logicalDirection,
                               float leadDistanceDeg,
                               uint8_t excludedBoundary,
                               uint8_t &boundaryIndex,
                               BrakePointApproach &approach,
                               float &distanceDeg) {
  bool found = false;
  float closestDistance = 361.0f;
  for (uint8_t candidate = 0;
       candidate < BP_BOUNDARY_COUNT;
       ++candidate) {
    if (candidate == excludedBoundary) {
      continue;
    }
    BrakePointApproach candidateApproach =
        BrakePointApproach::Inactive;
    float candidateDistance = 0.0f;
    if (brakePointBoundaryThreatens(
            axisIndex,
            candidate,
            currentAngleDeg,
            logicalDirection,
            leadDistanceDeg,
            candidateApproach,
            candidateDistance) &&
        candidateDistance < closestDistance) {
      found = true;
      closestDistance = candidateDistance;
      boundaryIndex = candidate;
      approach = candidateApproach;
      distanceDeg = candidateDistance;
    }
  }
  return found;
}

void setBrakePointFault(uint8_t axisIndex,
                        BrakePointFaultReason reason,
                        Direction debugDirection = Direction::Stop) {
  if (axisIndex >= 3) {
    return;
  }
  AxisRuntime &runtime = axisRuntime[axisIndex];
  runtime.brakePointPhase = BrakePointPhase::Fault;
  runtime.brakePointFaultReason = reason;
  runtime.brakePointPhaseStartedMs = millis();
  runtime.brakePointRecoveryOutputAuthorized = false;
  cancelTargets();
  clearAxisKeybindRuntime(axisIndex);
  for (uint8_t otherAxis = 0; otherAxis < 3; ++otherAxis) {
    if (otherAxis != axisIndex) {
      driveMotorHardware(otherAxis, Direction::Stop, 0);
    }
  }
  brakePointFaultBits |= static_cast<uint8_t>(1U << axisIndex);
  brakeMotor(axisIndex, BRAKE_PULSE_MS);
  debugBrakePoint(axisIndex, debugDirection, 0, 0, true);
}

void beginBrakePointStop(uint8_t axisIndex,
                         uint8_t boundaryIndex,
                         BrakePointApproach approach,
                         float leadDistanceDeg) {
  if (axisIndex >= 3 ||
      boundaryIndex >= BP_BOUNDARY_COUNT ||
      approach == BrakePointApproach::Inactive) {
    return;
  }
  AxisRuntime &runtime = axisRuntime[axisIndex];
  if (runtime.brakePointPhase != BrakePointPhase::Inactive) {
    return;
  }
  runtime.activeBrakePointBoundary = boundaryIndex;
  runtime.brakePointApproach = approach;
  runtime.brakePointPhase = BrakePointPhase::Braking;
  runtime.brakePointFaultReason = BrakePointFaultReason::None;
  runtime.brakePointLeadAtTriggerDeg = leadDistanceDeg;
  runtime.brakePointRecoveryTargetDeg = 0.0f;
  runtime.brakePointRecoveryDirection = Direction::Stop;
  runtime.brakePointPhaseStartedMs = millis();
  runtime.brakePointSettledSamples = 0;
  runtime.brakePointNeutralSeen = false;
  runtime.brakePointRetreatCommandSeen = false;
  runtime.brakePointRecoveryOutputAuthorized = false;
  cancelTargets();
  clearAxisKeybindRuntime(axisIndex);
  for (uint8_t otherAxis = 0; otherAxis < 3; ++otherAxis) {
    if (otherAxis != axisIndex) {
      driveMotorHardware(otherAxis, Direction::Stop, 0);
    }
  }
  brakePointFaultBits &= static_cast<uint8_t>(~(1U << axisIndex));
  brakeMotor(axisIndex, BRAKE_PULSE_MS);
  debugBrakePoint(axisIndex, Direction::Stop, 0, 0, true);
}

bool brakePointRecoveryPathIsClear(uint8_t axisIndex,
                                   uint8_t activeBoundary,
                                   float currentAngleDeg,
                                   float targetAngleDeg) {
  for (uint8_t boundaryIndex = 0;
       boundaryIndex < BP_BOUNDARY_COUNT;
       ++boundaryIndex) {
    if (boundaryIndex == activeBoundary) {
      continue;
    }
    BrakePointBoundaryView boundary;
    if (!getBrakePointBoundary(axisIndex, boundaryIndex, boundary)) {
      continue;
    }
    if (bpPathCrossesForbidden(
            boundary.mode,
            currentAngleDeg,
            targetAngleDeg,
            boundary.angleDeg)) {
      return false;
    }
  }
  return true;
}

void updateBrakePointSensorState(uint8_t axisIndex,
                                 float previousAngleDeg,
                                 float currentAngleDeg,
                                 bool hadPreviousAngle) {
  if (axisIndex >= 3) {
    return;
  }
  AxisRuntime &runtime = axisRuntime[axisIndex];
  if (mode != SystemMode::Arm ||
      !axisHasAnyBrakePoint(axisIndex) ||
      (sensorFaultBits & (1U << axisIndex)) != 0) {
    resetBrakePointRuntime(axisIndex);
    return;
  }
  if (runtime.brakePointPhase == BrakePointPhase::Fault) {
    return;
  }

  if (runtime.brakePointPhase == BrakePointPhase::Inactive) {
    const float leadDistance = brakeLeadDistanceDeg(axisIndex);
    if (hadPreviousAngle) {
      const float delta =
          shortestErrorDeg(currentAngleDeg, previousAngleDeg);
      const Direction movementDirection =
          delta >= 0.0f ? Direction::Forward : Direction::Backward;
      bool crossed = false;
      float closestCrossing = 361.0f;
      uint8_t crossedBoundary = BP_INVALID_BOUNDARY;
      BrakePointApproach crossedApproach =
          BrakePointApproach::Inactive;
      for (uint8_t boundaryIndex = 0;
           boundaryIndex < BP_BOUNDARY_COUNT;
           ++boundaryIndex) {
        BrakePointBoundaryView boundary;
        if (!getBrakePointBoundary(
                axisIndex, boundaryIndex, boundary)) {
          continue;
        }
        BrakePointApproach candidateApproach =
            BrakePointApproach::Inactive;
        if (!bpSegmentCrossedForMode(
                boundary.mode,
                previousAngleDeg,
                currentAngleDeg,
                boundary.angleDeg,
                candidateApproach)) {
          continue;
        }
        const float crossingDistance =
            bpDirectionalDistanceDeg(
                previousAngleDeg,
                boundary.angleDeg,
                static_cast<int8_t>(movementDirection));
        if (!crossed || crossingDistance < closestCrossing) {
          crossed = true;
          closestCrossing = crossingDistance;
          crossedBoundary = boundaryIndex;
          crossedApproach = candidateApproach;
        }
      }
      if (crossed) {
        beginBrakePointStop(
            axisIndex,
            crossedBoundary,
            crossedApproach,
            leadDistance);
        return;
      }
    }

    const AxisConfig &cfg = settings.axis[axisIndex];
    const Direction logicalMotorDirection =
        applyInvert(motors[axisIndex].direction, cfg.invert);
    uint8_t threateningBoundary = BP_INVALID_BOUNDARY;
    BrakePointApproach threateningApproach =
        BrakePointApproach::Inactive;
    float threatDistance = 0.0f;
    if (findThreateningBrakePoint(
            axisIndex,
            currentAngleDeg,
            logicalMotorDirection,
            leadDistance,
            BP_INVALID_BOUNDARY,
            threateningBoundary,
            threateningApproach,
            threatDistance)) {
      beginBrakePointStop(
          axisIndex,
          threateningBoundary,
          threateningApproach,
          leadDistance);
      return;
    }

    Direction encoderDirection = Direction::Stop;
    if (runtime.velocityDps > BP_APPROACH_MIN_VELOCITY_DPS) {
      encoderDirection = Direction::Forward;
    } else if (runtime.velocityDps < -BP_APPROACH_MIN_VELOCITY_DPS) {
      encoderDirection = Direction::Backward;
    }
    if (findThreateningBrakePoint(
            axisIndex,
            currentAngleDeg,
            encoderDirection,
            leadDistance,
            BP_INVALID_BOUNDARY,
            threateningBoundary,
            threateningApproach,
            threatDistance)) {
      beginBrakePointStop(
          axisIndex,
          threateningBoundary,
          threateningApproach,
          leadDistance);
    }
    return;
  }

  BrakePointBoundaryView activeBoundary;
  if (!getBrakePointBoundary(
          axisIndex,
          runtime.activeBrakePointBoundary,
          activeBoundary)) {
    setBrakePointFault(
        axisIndex, BrakePointFaultReason::RuntimeConflict);
    return;
  }

  const uint32_t now = millis();
  if (runtime.brakePointPhase == BrakePointPhase::Braking) {
    if (!motors[axisIndex].braking) {
      runtime.brakePointPhase = BrakePointPhase::Settling;
      runtime.brakePointPhaseStartedMs = now;
      runtime.brakePointSettledSamples = 0;
      debugBrakePoint(axisIndex, Direction::Stop, 0, 0, true);
    }
    return;
  }

  if (runtime.brakePointPhase == BrakePointPhase::Settling) {
    const uint32_t elapsedMs = now - runtime.brakePointPhaseStartedMs;
    if (elapsedMs >= BP_SETTLE_MIN_MS &&
        fabsf(runtime.velocityDps) <= BP_SETTLE_VELOCITY_DPS) {
      if (runtime.brakePointSettledSamples < UINT8_MAX) {
        ++runtime.brakePointSettledSamples;
      }
    } else {
      runtime.brakePointSettledSamples = 0;
    }

    if (runtime.brakePointSettledSamples >= BP_SETTLE_SAMPLE_COUNT) {
      float originalSideDistance = 0.0f;
      const bool onOriginalSide = brakePointOnOriginalSide(
          axisIndex, currentAngleDeg, originalSideDistance);
      const bool recoveryRequired = !onOriginalSide;
      if (recoveryRequired &&
          settings.brakePoint[axisIndex].recoveryMaxPwm > 0) {
        const float clearance =
            brakePointEffectiveClearanceDeg(axisIndex) +
            brakePointRecoveryToleranceDeg(axisIndex);
        runtime.brakePointRecoveryTargetDeg = normalize360(
            activeBoundary.angleDeg +
            (runtime.brakePointApproach ==
                     BrakePointApproach::FromBelow
                 ? -clearance
                 : clearance));
        runtime.brakePointRecoveryDirection =
            runtime.brakePointApproach ==
                    BrakePointApproach::FromBelow
                ? Direction::Backward
                : Direction::Forward;
        if (!brakePointRecoveryPathIsClear(
                axisIndex,
                runtime.activeBrakePointBoundary,
                currentAngleDeg,
                runtime.brakePointRecoveryTargetDeg)) {
          setBrakePointFault(
              axisIndex, BrakePointFaultReason::RecoveryBlocked);
          return;
        }
        runtime.brakePointPhase = BrakePointPhase::Recovering;
        runtime.brakePointPhaseStartedMs = now;
        runtime.brakePointRecoveryLastDistanceDeg =
            bpDirectionalDistanceDeg(
                currentAngleDeg,
                runtime.brakePointRecoveryTargetDeg,
                static_cast<int8_t>(
                    runtime.brakePointRecoveryDirection));
        runtime.brakePointRecoveryLastProgressMs = now;
        resetPositionControllerRuntime(axisIndex);
      } else {
        runtime.brakePointPhase = BrakePointPhase::Guarded;
        runtime.brakePointPhaseStartedMs = now;
      }
      debugBrakePoint(axisIndex, Direction::Stop, 0, 0, true);
      return;
    }

    if (elapsedMs >= BP_SETTLE_REBRAKE_MS) {
      runtime.brakePointPhase = BrakePointPhase::Braking;
      runtime.brakePointPhaseStartedMs = now;
      runtime.brakePointSettledSamples = 0;
      brakeMotor(axisIndex, BRAKE_PULSE_MS);
      debugBrakePoint(axisIndex, Direction::Stop, 0, 0, true);
    }
    return;
  }

  if (runtime.brakePointPhase == BrakePointPhase::Guarded) {
    float originalSideDistance = 0.0f;
    const bool safelyOutside =
        brakePointOnOriginalSide(axisIndex, currentAngleDeg, originalSideDistance) &&
        originalSideDistance >= brakePointEffectiveClearanceDeg(axisIndex);
    if (safelyOutside &&
        (runtime.brakePointNeutralSeen || runtime.brakePointRetreatCommandSeen)) {
      resetBrakePointRuntime(axisIndex);
      debugBrakePoint(axisIndex, Direction::Stop, 0, 0, true);
    }
  }
}

BrakePointOutputAction filterBrakePointMotorOutput(uint8_t axisIndex,
                                                   Direction physicalDirection,
                                                   uint8_t requestedPwm) {
  if (axisIndex >= 3 ||
      mode != SystemMode::Arm ||
      !axisHasAnyBrakePoint(axisIndex) ||
      physicalDirection == Direction::Stop || requestedPwm == 0) {
    return BrakePointOutputAction::Allow;
  }

  AxisRuntime &runtime = axisRuntime[axisIndex];
  const AxisConfig &cfg = settings.axis[axisIndex];
  const Direction logicalDirection = applyInvert(physicalDirection, cfg.invert);
  const uint32_t nowUs = GetMicroseconds();
  const bool freshAngle =
      (sensorFaultBits & (1U << axisIndex)) == 0 &&
      runtime.angleValid &&
      runtime.lastSampleUs != 0 &&
      nowUs - runtime.lastSampleUs <= AS5600_STALE_TIMEOUT_US;
  if (!freshAngle) {
    // Sensor-dependent features are temporarily bypassed. Direct manual PWM
    // remains usable and becomes BP-protected again after a fresh sample.
    return BrakePointOutputAction::Allow;
  }

  if (runtime.brakePointPhase == BrakePointPhase::Fault ||
      runtime.brakePointPhase == BrakePointPhase::Braking ||
      runtime.brakePointPhase == BrakePointPhase::Settling) {
    debugBrakePoint(axisIndex, logicalDirection, requestedPwm, 0);
    return BrakePointOutputAction::Ignore;
  }

  if (runtime.brakePointPhase == BrakePointPhase::Recovering) {
    BrakePointBoundaryView activeBoundary;
    if (!runtime.brakePointRecoveryOutputAuthorized ||
        !getBrakePointBoundary(
            axisIndex,
            runtime.activeBrakePointBoundary,
            activeBoundary) ||
        logicalDirection != runtime.brakePointRecoveryDirection ||
        brakePointDirectionForbidden(
            activeBoundary.mode,
            runtime.brakePointApproach,
            logicalDirection)) {
      debugBrakePoint(axisIndex, logicalDirection, requestedPwm, 0);
      return BrakePointOutputAction::Ignore;
    }
    uint8_t otherBoundary = BP_INVALID_BOUNDARY;
    BrakePointApproach otherApproach =
        BrakePointApproach::Inactive;
    float otherDistance = 0.0f;
    if (findThreateningBrakePoint(
            axisIndex,
            runtime.angleDeg,
            logicalDirection,
            brakeLeadDistanceDeg(axisIndex),
            runtime.activeBrakePointBoundary,
            otherBoundary,
            otherApproach,
            otherDistance)) {
      setBrakePointFault(
          axisIndex,
          BrakePointFaultReason::RuntimeConflict,
          logicalDirection);
      return BrakePointOutputAction::Brake;
    }
    debugBrakePoint(axisIndex, logicalDirection, requestedPwm, requestedPwm);
    return BrakePointOutputAction::Allow;
  }

  if (runtime.brakePointPhase == BrakePointPhase::Inactive) {
    const float leadDistance = brakeLeadDistanceDeg(axisIndex);
    uint8_t boundaryIndex = BP_INVALID_BOUNDARY;
    BrakePointApproach approach =
        BrakePointApproach::Inactive;
    float distanceDeg = 0.0f;
    if (findThreateningBrakePoint(
            axisIndex,
            runtime.angleDeg,
            logicalDirection,
            leadDistance,
            BP_INVALID_BOUNDARY,
            boundaryIndex,
            approach,
            distanceDeg)) {
      beginBrakePointStop(
          axisIndex, boundaryIndex, approach, leadDistance);
      debugBrakePoint(axisIndex, logicalDirection, requestedPwm, 0, true);
      return BrakePointOutputAction::Brake;
    }
    return BrakePointOutputAction::Allow;
  }

  if (runtime.brakePointPhase == BrakePointPhase::Guarded) {
    BrakePointBoundaryView activeBoundary;
    if (!getBrakePointBoundary(
            axisIndex,
            runtime.activeBrakePointBoundary,
            activeBoundary)) {
      setBrakePointFault(
          axisIndex, BrakePointFaultReason::RuntimeConflict);
      return BrakePointOutputAction::Brake;
    }
    if (brakePointDirectionForbidden(
            activeBoundary.mode,
            runtime.brakePointApproach,
            logicalDirection)) {
      targets[axisIndex].active = false;
      targets[axisIndex].hold = false;
      resetPositionControllerRuntime(axisIndex);
      debugBrakePoint(axisIndex, logicalDirection, requestedPwm, 0);
      return BrakePointOutputAction::Brake;
    }
    uint8_t otherBoundary = BP_INVALID_BOUNDARY;
    BrakePointApproach otherApproach =
        BrakePointApproach::Inactive;
    float otherDistance = 0.0f;
    if (findThreateningBrakePoint(
            axisIndex,
            runtime.angleDeg,
            logicalDirection,
            brakeLeadDistanceDeg(axisIndex),
            runtime.activeBrakePointBoundary,
            otherBoundary,
            otherApproach,
            otherDistance)) {
      setBrakePointFault(
          axisIndex,
          BrakePointFaultReason::RuntimeConflict,
          logicalDirection);
      return BrakePointOutputAction::Brake;
    }
    runtime.brakePointRetreatCommandSeen = true;
    debugBrakePoint(axisIndex, logicalDirection, requestedPwm, requestedPwm);
  }
  return BrakePointOutputAction::Allow;
}

void drivePositionControllerOutput(uint8_t axisIndex, Direction direction, uint8_t desiredPwm) {
  if (axisIndex >= 3 || direction == Direction::Stop || desiredPwm == 0) {
    coastPositionControllerAxis(axisIndex);
    return;
  }

  AxisRuntime &runtime = axisRuntime[axisIndex];
  const AxisConfig &cfg = settings.axis[axisIndex];
  const uint32_t now = millis();

  if (runtime.pidReverseHoldUntilMs != 0 &&
      static_cast<int32_t>(now - runtime.pidReverseHoldUntilMs) < 0) {
    coastMotor(axisIndex);
    return;
  }
  if (runtime.pidReverseHoldUntilMs != 0) {
    runtime.pidReverseHoldUntilMs = 0;
  }

  if (runtime.pidOutputDirection != Direction::Stop &&
      runtime.pidOutputDirection != direction &&
      runtime.pidOutputPwm > 0) {
    coastMotor(axisIndex);
    runtime.pidOutputDirection = Direction::Stop;
    runtime.pidOutputPwm = 0;
    runtime.pidLastMs = now;
    runtime.pidReverseHoldUntilMs = now + POSITION_REVERSE_COAST_MS;
    return;
  }

  const uint32_t dtMs = runtime.pidLastMs == 0 ? 20 : now - runtime.pidLastMs;
  runtime.pidLastMs = now;
  const uint8_t rampedPwm = rampPwmToward(runtime.pidOutputPwm, desiredPwm, dtMs);
  runtime.pidOutputDirection = direction;
  runtime.pidOutputPwm = rampedPwm;
  driveMotorRaw(axisIndex, applyInvert(direction, cfg.invert), rampedPwm);
}

bool serviceBrakePointRecovery(uint8_t axisIndex, float currentAngleDeg) {
  if (axisIndex >= 3) {
    return false;
  }
  AxisRuntime &runtime = axisRuntime[axisIndex];
  if (runtime.brakePointPhase == BrakePointPhase::Inactive ||
      runtime.brakePointPhase == BrakePointPhase::Guarded) {
    return false;
  }
  if (runtime.brakePointPhase == BrakePointPhase::Braking ||
      runtime.brakePointPhase == BrakePointPhase::Settling ||
      runtime.brakePointPhase == BrakePointPhase::Fault) {
    return true;
  }
  if (runtime.brakePointPhase != BrakePointPhase::Recovering) {
    return false;
  }

  const uint32_t now = millis();
  const BrakePointTuning &tuning = settings.brakePoint[axisIndex];
  const float shortestTargetError =
      shortestErrorDeg(runtime.brakePointRecoveryTargetDeg,
                       currentAngleDeg);
  const float absError = fabsf(shortestTargetError);
  const float tolerance = brakePointRecoveryToleranceDeg(axisIndex);

  if (absError + BP_RECOVERY_PROGRESS_DEG <
      runtime.brakePointRecoveryLastDistanceDeg) {
    runtime.brakePointRecoveryLastDistanceDeg = absError;
    runtime.brakePointRecoveryLastProgressMs = now;
  }

  const bool timedOut =
      now - runtime.brakePointPhaseStartedMs >= tuning.recoveryTimeoutMs;
  const bool noProgress =
      now - runtime.brakePointRecoveryLastProgressMs >=
      BP_RECOVERY_NO_PROGRESS_MS;
  if (timedOut || noProgress) {
    setBrakePointFault(
        axisIndex,
        timedOut ? BrakePointFaultReason::RecoveryTimeout
                 : BrakePointFaultReason::RecoveryNoProgress);
    return true;
  }

  if (absError <= tolerance &&
      fabsf(runtime.velocityDps) <= BP_SETTLE_VELOCITY_DPS) {
    runtime.brakePointRecoveryOutputAuthorized = false;
    coastPositionControllerAxis(axisIndex);
    resetPositionControllerRuntime(axisIndex);
    runtime.brakePointPhase = BrakePointPhase::Guarded;
    runtime.brakePointPhaseStartedMs = now;
    debugBrakePoint(axisIndex, Direction::Stop, 0, 0, true);
    return true;
  }

  const Direction direction = runtime.brakePointRecoveryDirection;
  BrakePointBoundaryView activeBoundary;
  if (direction == Direction::Stop ||
      !getBrakePointBoundary(
          axisIndex,
          runtime.activeBrakePointBoundary,
          activeBoundary)) {
    setBrakePointFault(
        axisIndex, BrakePointFaultReason::RuntimeConflict);
    return true;
  }
  const bool recoveryTargetPassed =
      (direction == Direction::Forward &&
       shortestTargetError < -tolerance) ||
      (direction == Direction::Backward &&
       shortestTargetError > tolerance);
  if (recoveryTargetPassed) {
    float originalSideDistance = 0.0f;
    if (brakePointOnOriginalSide(
            axisIndex, currentAngleDeg, originalSideDistance) &&
        originalSideDistance >=
            brakePointEffectiveClearanceDeg(axisIndex)) {
      runtime.brakePointRecoveryOutputAuthorized = false;
      coastPositionControllerAxis(axisIndex);
      resetPositionControllerRuntime(axisIndex);
      runtime.brakePointPhase = BrakePointPhase::Guarded;
      runtime.brakePointPhaseStartedMs = now;
      debugBrakePoint(
          axisIndex, Direction::Stop, 0, 0, true);
      return true;
    }
    setBrakePointFault(
        axisIndex, BrakePointFaultReason::RuntimeConflict);
    return true;
  }
  if (brakePointDirectionForbidden(
          activeBoundary.mode,
          runtime.brakePointApproach,
          direction)) {
    float originalSideDistance = 0.0f;
    const bool safelyOutside =
        brakePointOnOriginalSide(axisIndex, currentAngleDeg,
                                 originalSideDistance) &&
        originalSideDistance >= brakePointEffectiveClearanceDeg(axisIndex);
    if (safelyOutside) {
      runtime.brakePointRecoveryOutputAuthorized = false;
      coastPositionControllerAxis(axisIndex);
      resetPositionControllerRuntime(axisIndex);
      runtime.brakePointPhase = BrakePointPhase::Guarded;
      runtime.brakePointPhaseStartedMs = now;
      debugBrakePoint(axisIndex, Direction::Stop, 0, 0, true);
      return true;
    }
    setBrakePointFault(
        axisIndex,
        BrakePointFaultReason::RecoveryBlocked,
        direction);
    return true;
  }
  if (!brakePointRecoveryPathIsClear(
          axisIndex,
          runtime.activeBrakePointBoundary,
          currentAngleDeg,
          runtime.brakePointRecoveryTargetDeg)) {
    setBrakePointFault(
        axisIndex,
        BrakePointFaultReason::RecoveryBlocked,
        direction);
    return true;
  }

  const AxisConfig &cfg = settings.axis[axisIndex];
  const float error =
      direction == Direction::Forward ? absError : -absError;
  const float controlSigned =
      (error * static_cast<float>(cfg.kp)) / 1000.0f -
      (runtime.velocityDps * static_cast<float>(cfg.kd)) / 1000.0f;
  if ((direction == Direction::Forward && controlSigned <= 0.0f) ||
      (direction == Direction::Backward && controlSigned >= 0.0f)) {
    runtime.brakePointRecoveryOutputAuthorized = true;
    coastPositionControllerAxis(axisIndex);
    runtime.brakePointRecoveryOutputAuthorized = false;
    return true;
  }
  int pwm = static_cast<int>(fabsf(controlSigned));
  if (pwm < cfg.minPwm) {
    pwm = cfg.minPwm;
  }
  const uint8_t recoveryLimit =
      tuning.recoveryMaxPwm < cfg.maxPwm
          ? tuning.recoveryMaxPwm
          : cfg.maxPwm;
  const uint8_t desiredPwm = clampPwm(pwm, recoveryLimit);
  runtime.brakePointRecoveryOutputAuthorized = true;
  drivePositionControllerOutput(axisIndex, direction, desiredPwm);
  runtime.brakePointRecoveryOutputAuthorized = false;
  debugBrakePoint(axisIndex, direction, desiredPwm, desiredPwm);
  return true;
}

void MotionControl_UpdateFast() {
  if (mode != SystemMode::Arm) {
    return;
  }

  for (uint8_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
    if (motors[axisIndex].braking) {
      continue;
    }

    float current = 0.0f;
    if (!getFreshAxisAngle(axisIndex, current)) {
      if (targets[axisIndex].active) {
        targets[axisIndex].active = false;
        targets[axisIndex].hold = false;
        coastPositionControllerAxis(axisIndex);
        resetPositionControllerRuntime(axisIndex);
      }
      continue;
    }

    if (serviceBrakePointRecovery(axisIndex, current)) {
      continue;
    }

    if (!targets[axisIndex].active) {
      continue;
    }

    const AxisConfig &cfg = settings.axis[axisIndex];
    const float error = shortestErrorDeg(targets[axisIndex].targetDeg, current);
    const float tolerance = cfg.toleranceDeg > 0.0f ? cfg.toleranceDeg : POSITION_TOLERANCE_DEG;
    const float absError = fabsf(error);
    const float absVelocity = fabsf(axisRuntime[axisIndex].velocityDps);
    if (targets[axisIndex].hold) {
      const float reengageTolerance = fmaxf(tolerance * HOLD_REENGAGE_MULTIPLIER,
                                            tolerance + HOLD_REENGAGE_MIN_DEG);
      if (absError <= tolerance && absVelocity <= HOLD_SETTLE_VELOCITY_DPS) {
        coastPositionControllerAxis(axisIndex);
        axisRuntime[axisIndex].holdSettled = true;
        continue;
      }
      if (axisRuntime[axisIndex].holdSettled &&
          absError < reengageTolerance &&
          absVelocity <= HOLD_SETTLE_VELOCITY_DPS) {
        coastPositionControllerAxis(axisIndex);
        continue;
      }
      axisRuntime[axisIndex].holdSettled = false;
    } else if (absError <= tolerance) {
        targets[axisIndex].active = false;
        targets[axisIndex].hold = false;
        resetPositionControllerRuntime(axisIndex);
        stopAxis(axisIndex + 1);
      continue;
    }

    const float controlSigned = (error * static_cast<float>(cfg.kp)) / 1000.0f -
                                (axisRuntime[axisIndex].velocityDps * static_cast<float>(cfg.kd)) / 1000.0f;
    Direction direction = Direction::Stop;
    if (controlSigned > 0.0f) {
      direction = Direction::Forward;
    } else if (controlSigned < 0.0f) {
      direction = Direction::Backward;
    } else {
      direction = error > 0.0f ? Direction::Forward : Direction::Backward;
    }
    int pwm = static_cast<int>(fabsf(controlSigned));
    if (pwm < cfg.minPwm) {
      pwm = cfg.minPwm;
    }
    drivePositionControllerOutput(axisIndex, direction, clampPwm(pwm, cfg.maxPwm));
  }
}

void serviceStream() {
  if (!streamAS5600Active) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastStreamMs < streamIntervalMs) {
    return;
  }
  lastStreamMs = now;

  bool foundAny = false;
  printAS5600Channel(streamAS5600Channel, foundAny);
  if (!foundAny) {
    sendErr("AS5600_NOT_FOUND");
  }
}

void Telemetry_Update() {
  const uint32_t now = millis();
  if (now - lastStatusMs < statusIntervalMs) {
    return;
  }
  lastStatusMs = now;
  printStatusTelemetry();
  printDrillAccessoryTelemetry();
  ++telemetry_send_count;
}

bool fastTaskDue(uint32_t nowUs, uint32_t &lastRunUs, uint32_t periodUs) {
  if (nowUs - lastRunUs < periodUs) {
    return false;
  }

  lastRunUs += periodUs;
  if (nowUs - lastRunUs >= periodUs) {
    // Do not execute catch-up bursts after UART/I2C stalls; resume from now.
    lastRunUs = nowUs;
  }
  return true;
}

void serviceFastTasks() {
  uint32_t nowUs = GetMicroseconds();
  if (fastTaskDue(nowUs, lastAS5600UpdateUs, AS5600_UPDATE_SLOT_PERIOD_US)) {
    AS5600_UpdateFast();
  }

  nowUs = GetMicroseconds();
  if (fastTaskDue(nowUs, lastMotionControlUs, MOTION_CONTROL_PERIOD_US)) {
    MotionControl_UpdateFast();
  }
}

void serviceHardwareWatchdog(uint8_t completedHealthStages) {
  if (!hardwareWatchdogStarted ||
      completedHealthStages != LOOP_HEALTH_REQUIRED) {
    return;
  }

  const uint32_t nowMs = millis();
  if (!bootResetStreakCleared &&
      nowMs >= BOOT_STREAK_CLEAR_AFTER_MS) {
    enableBackupRegisterWrites();
    volatile uint32_t *backup = faultBackupRegisters();
    backup[BOOT_STREAK_COUNT_INDEX] = 0;
    bootResetStreak = 0;
    bootResetStreakCleared = true;
  }
  if (nowMs - hardwareWatchdogLastReloadMs <
      HARDWARE_WATCHDOG_RELOAD_INTERVAL_MS) {
    return;
  }

  // Reload only after serial, safety/output, fast-control and telemetry stages
  // have all returned. A block in any stage therefore expires the IWDG.
  IWatchdog.reload();
  hardwareWatchdogLastReloadMs = nowMs;
}

const char *cpuFaultReasonName(uint16_t reason) {
  switch (static_cast<CpuFaultReason>(reason)) {
    case CpuFaultReason::HardFault:
      return "HARDFAULT";
    case CpuFaultReason::MemManage:
      return "MEMMANAGE";
    case CpuFaultReason::BusFault:
      return "BUSFAULT";
    case CpuFaultReason::UsageFault:
      return "USAGEFAULT";
    default:
      return "UNKNOWN";
  }
}

bool printAndClearCpuFaultRecord() {
  enableBackupRegisterWrites();
  volatile uint32_t *record = faultBackupRegisters();
  if (record[0] != CPU_FAULT_RECORD_MAGIC) {
    return false;
  }

  const uint32_t versionAndReason = record[1];
  const uint16_t version = static_cast<uint16_t>(versionAndReason >> 16);
  const uint16_t reason = static_cast<uint16_t>(versionAndReason);
  const bool valid =
      version == CPU_FAULT_RECORD_VERSION &&
      record[CPU_FAULT_CHECKSUM_INDEX] == faultRecordChecksum(record);

  if (valid) {
    LinkSerial.print("OK BOOT CPU_FAULT TYPE=");
    LinkSerial.print(cpuFaultReasonName(reason));
    LinkSerial.print(" PC=0x");
    LinkSerial.print(record[2], HEX);
    LinkSerial.print(" LR=0x");
    LinkSerial.print(record[3], HEX);
    LinkSerial.print(" CFSR=0x");
    LinkSerial.print(record[6], HEX);
    LinkSerial.print(" HFSR=0x");
    LinkSerial.print(record[7], HEX);
    LinkSerial.print(" MMFAR=0x");
    LinkSerial.print(record[10], HEX);
    LinkSerial.print(" BFAR=0x");
    LinkSerial.println(record[11], HEX);
  } else {
    LinkSerial.println("ERR BOOT CPU_FAULT_RECORD_CORRUPT");
  }

  // Consume the record only after it has been copied to the UART buffer.
  record[0] = 0;
  record[CPU_FAULT_CHECKSUM_INDEX] = 0;
  __DSB();
  return valid;
}

void updateAndPrintBootResetStreak(bool cpuFaultRecorded) {
  enableBackupRegisterWrites();
  volatile uint32_t *backup = faultBackupRegisters();
#ifdef RCC_CSR_IWDGRSTF
  const bool watchdogReset =
      (bootResetFlags & RCC_CSR_IWDGRSTF) != 0U;
#else
  const bool watchdogReset = false;
#endif
  const bool abnormalReset = watchdogReset || cpuFaultRecorded;

  if (abnormalReset) {
    uint32_t previous = 0;
    if (backup[BOOT_STREAK_MAGIC_INDEX] == BOOT_STREAK_MAGIC) {
      previous = backup[BOOT_STREAK_COUNT_INDEX];
    }
    bootResetStreak = previous == UINT32_MAX ? UINT32_MAX : previous + 1U;
  } else {
    bootResetStreak = 0;
  }
  backup[BOOT_STREAK_COUNT_INDEX] = bootResetStreak;
  backup[BOOT_STREAK_MAGIC_INDEX] = BOOT_STREAK_MAGIC;
  __DSB();

  if (bootResetStreak > 0) {
    LinkSerial.print("OK BOOT RESET_STREAK=");
    LinkSerial.println(bootResetStreak);
  }
}

void printBootResetCause() {
  LinkSerial.print("OK BOOT RESET_FLAGS=0x");
  LinkSerial.print(bootResetFlags, HEX);
#ifdef RCC_CSR_BORRSTF
  if ((bootResetFlags & RCC_CSR_BORRSTF) != 0U) {
    LinkSerial.print(" BOR");
  }
#endif
#ifdef RCC_CSR_PORRSTF
  if ((bootResetFlags & RCC_CSR_PORRSTF) != 0U) {
    LinkSerial.print(" POR");
  }
#endif
#ifdef RCC_CSR_PINRSTF
  if ((bootResetFlags & RCC_CSR_PINRSTF) != 0U) {
    LinkSerial.print(" PIN");
  }
#endif
#ifdef RCC_CSR_SFTRSTF
  if ((bootResetFlags & RCC_CSR_SFTRSTF) != 0U) {
    LinkSerial.print(" SOFTWARE");
  }
#endif
#ifdef RCC_CSR_IWDGRSTF
  if ((bootResetFlags & RCC_CSR_IWDGRSTF) != 0U) {
    LinkSerial.print(" IWDG");
  }
#endif
#ifdef RCC_CSR_WWDGRSTF
  if ((bootResetFlags & RCC_CSR_WWDGRSTF) != 0U) {
    LinkSerial.print(" WWDG");
  }
#endif
#ifdef RCC_CSR_LPWRRSTF
  if ((bootResetFlags & RCC_CSR_LPWRRSTF) != 0U) {
    LinkSerial.print(" LOW_POWER");
  }
#endif
  LinkSerial.println();
#ifdef RCC_CSR_RMVF
  RCC->CSR |= RCC_CSR_RMVF;
#endif
}

void setup() {
  bootResetFlags = RCC->CSR;
  configureDrillAccessoryPins();
  configureMotorPins();
  SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk |
                SCB_SHCSR_BUSFAULTENA_Msk |
                SCB_SHCSR_USGFAULTENA_Msk;
  initializeMicrosecondClock();
  debugWindowStartMs = millis();

  LinkSerial.setRx(PA_3);
  LinkSerial.setTx(PA_2);
  LinkSerial.begin(LINK_BAUD);
  delay(50);
  printBootResetCause();
  const bool cpuFaultRecorded = printAndClearCpuFaultRecord();
  updateAndPrintBootResetStreak(cpuFaultRecorded);
  sendOk("BOOT UART READY");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(80);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(80);
  sensors.begin();
  sendOk("BOOT I2C READY");
  loadSettings();
  if (settingsLoadedFromFlash) {
    sendOk("BOOT SETTINGS FLASH");
  } else {
    sendOk("BOOT SETTINGS DEFAULT_RAM");
  }
  sendOk("BOOT SETTINGS READY");
  stopAll(false);
  mode = SystemMode::Safe;
  servoRuntime.safetyReason = ServoSafetyReason::Boot;
  const uint32_t nowUs = GetMicroseconds();
  lastAS5600UpdateUs = nowUs;
  lastMotionControlUs = nowUs;
  sendOk("BOOT MODE SAFE");
  LinkSerial.flush();
  IWatchdog.begin(HARDWARE_WATCHDOG_TIMEOUT_US);
  hardwareWatchdogStarted = true;
  hardwareWatchdogLastReloadMs = millis();
  sendOk("BOOT IWDG READY TIMEOUT_MS=4000");
}

void loop() {
  uint8_t healthStages = 0;

  serviceSerial();
  healthStages |= LOOP_HEALTH_SERIAL;

  serviceBrakeTimers();
  serviceServo();
  serviceSensorBusRecovery();
  healthStages |= LOOP_HEALTH_SAFETY;

  serviceFastTasks();
  healthStages |= LOOP_HEALTH_FAST_TASKS;

  Telemetry_Update();
  serviceStream();
  serviceDebugCounters();
  healthStages |= LOOP_HEALTH_TELEMETRY;

  serviceHardwareWatchdog(healthStages);
}
