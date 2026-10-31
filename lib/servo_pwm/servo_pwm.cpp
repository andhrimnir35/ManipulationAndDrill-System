#include "servo_pwm.h"

namespace {

constexpr uint32_t SERVO_TIMER_TICK_HZ = 1000000UL;
constexpr uint32_t SERVO_GPIO_PIN_INDEX = 0;
constexpr uint32_t SERVO_GPIO_AF = 2;
constexpr uint32_t SERVO_PWM_MODE_1 = 6;

bool outputEnabled = false;
uint16_t commandedPulseUs = 1500;
uint32_t writeCount = 0;
uint32_t lastChangeMs = 0;
ServoPulseSource lastSource = ServoPulseSource::None;

uint32_t timerClockHz() {
  uint32_t clockHz = HAL_RCC_GetPCLK1Freq();
  // On STM32F4 the APB timer clock is doubled whenever APB1 is prescaled.
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) {
    clockHz *= 2U;
  }
  return clockHz;
}

bool timerParameters(uint32_t &clockHz, uint32_t &prescaler) {
  clockHz = timerClockHz();
  if (clockHz == 0U || clockHz % SERVO_TIMER_TICK_HZ != 0U) {
    return false;
  }
  prescaler = clockHz / SERVO_TIMER_TICK_HZ;
  return prescaler >= 1U && prescaler <= 65536U;
}

void configurePa0AsTim5Ch1() {
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIOA->MODER =
      (GPIOA->MODER & ~(3UL << (SERVO_GPIO_PIN_INDEX * 2U))) |
      (2UL << (SERVO_GPIO_PIN_INDEX * 2U));
  GPIOA->OTYPER &= ~(1UL << SERVO_GPIO_PIN_INDEX);
  GPIOA->OSPEEDR =
      (GPIOA->OSPEEDR & ~(3UL << (SERVO_GPIO_PIN_INDEX * 2U))) |
      (2UL << (SERVO_GPIO_PIN_INDEX * 2U));
  GPIOA->PUPDR &= ~(3UL << (SERVO_GPIO_PIN_INDEX * 2U));
  GPIOA->AFR[0] =
      (GPIOA->AFR[0] & ~(0xFUL << (SERVO_GPIO_PIN_INDEX * 4U))) |
      (SERVO_GPIO_AF << (SERVO_GPIO_PIN_INDEX * 4U));
}

void holdPa0LowAsGpio() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIOA->BSRR = 1UL << (SERVO_GPIO_PIN_INDEX + 16U);
  GPIOA->MODER =
      (GPIOA->MODER & ~(3UL << (SERVO_GPIO_PIN_INDEX * 2U))) |
      (1UL << (SERVO_GPIO_PIN_INDEX * 2U));
  GPIOA->OTYPER &= ~(1UL << SERVO_GPIO_PIN_INDEX);
  GPIOA->PUPDR &= ~(3UL << (SERVO_GPIO_PIN_INDEX * 2U));
}

bool configureTimer(uint16_t pulseUs) {
  uint32_t clockHz = 0;
  uint32_t prescaler = 0;
  if (!timerParameters(clockHz, prescaler)) {
    return false;
  }

  __HAL_RCC_TIM5_CLK_ENABLE();
  TIM5->CR1 = 0;
  TIM5->CR2 = 0;
  TIM5->SMCR = 0;
  TIM5->DIER = 0;
  TIM5->CCER = 0;
  TIM5->CCMR1 = 0;
  TIM5->CCMR2 = 0;
  TIM5->CNT = 0;
  TIM5->PSC = prescaler - 1U;
  TIM5->ARR = SERVO_PWM_PERIOD_US - 1U;
  TIM5->CCR1 = pulseUs;
  TIM5->CCMR1 =
      (SERVO_PWM_MODE_1 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
  TIM5->CCER = TIM_CCER_CC1E;
  TIM5->EGR = TIM_EGR_UG;
  TIM5->SR = 0;

  configurePa0AsTim5Ch1();
  TIM5->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
  return true;
}

bool configurationValid() {
  if (!outputEnabled) {
    return (TIM5->CR1 & TIM_CR1_CEN) == 0U &&
           (TIM5->CCER & TIM_CCER_CC1E) == 0U &&
           ((GPIOA->MODER >> (SERVO_GPIO_PIN_INDEX * 2U)) & 3U) == 1U;
  }

  uint32_t clockHz = 0;
  uint32_t prescaler = 0;
  if (!timerParameters(clockHz, prescaler)) {
    return false;
  }
  const uint32_t mode =
      (GPIOA->MODER >> (SERVO_GPIO_PIN_INDEX * 2U)) & 3U;
  const uint32_t alternateFunction =
      (GPIOA->AFR[0] >> (SERVO_GPIO_PIN_INDEX * 4U)) & 0xFU;
  const uint32_t outputCompareMode =
      (TIM5->CCMR1 & TIM_CCMR1_OC1M) >> TIM_CCMR1_OC1M_Pos;

  return mode == 2U &&
         alternateFunction == SERVO_GPIO_AF &&
         TIM5->PSC == prescaler - 1U &&
         TIM5->ARR == SERVO_PWM_PERIOD_US - 1U &&
         TIM5->CCR1 == commandedPulseUs &&
         outputCompareMode == SERVO_PWM_MODE_1 &&
         (TIM5->CCMR1 & TIM_CCMR1_OC1PE) != 0U &&
         (TIM5->CCER & TIM_CCER_CC1E) != 0U &&
         (TIM5->CR1 & TIM_CR1_CEN) != 0U &&
         TIM5->DIER == 0U;
}

}  // namespace

bool servoPwmWriteMicroseconds(uint16_t pulseUs, ServoPulseSource source) {
  if (pulseUs < SERVO_PWM_MIN_US || pulseUs > SERVO_PWM_MAX_US) {
    return false;
  }

  if (!outputEnabled) {
    if (!configureTimer(pulseUs)) {
      return false;
    }
    outputEnabled = true;
  } else {
    if (!configurationValid()) {
      servoPwmStop();
      return false;
    }
    if (pulseUs == commandedPulseUs) {
      return true;
    }
    // OC1 preload transfers this single write on the next 20 ms update event.
    TIM5->CCR1 = pulseUs;
  }

  commandedPulseUs = pulseUs;
  lastSource = source;
  lastChangeMs = millis();
  ++writeCount;
  return configurationValid();
}

void servoPwmStop() {
  __HAL_RCC_TIM5_CLK_ENABLE();
  TIM5->DIER = 0;
  TIM5->CCER &= ~TIM_CCER_CC1E;
  TIM5->CR1 &= ~TIM_CR1_CEN;
  holdPa0LowAsGpio();
  outputEnabled = false;
  lastSource = ServoPulseSource::None;
}

ServoPwmDiagnostics servoPwmDiagnostics() {
  uint32_t clockHz = 0;
  uint32_t ignoredPrescaler = 0;
  timerParameters(clockHz, ignoredPrescaler);

  ServoPwmDiagnostics result = {};
  result.outputEnabled = outputEnabled;
  result.configurationValid = configurationValid();
  result.timerClockHz = clockHz;
  result.prescaler = TIM5->PSC;
  result.autoReload = TIM5->ARR;
  result.compare = TIM5->CCR1;
  result.counter = TIM5->CNT;
  result.interruptEnable = TIM5->DIER;
  result.gpioMode =
      (GPIOA->MODER >> (SERVO_GPIO_PIN_INDEX * 2U)) & 3U;
  result.gpioAlternateFunction =
      (GPIOA->AFR[0] >> (SERVO_GPIO_PIN_INDEX * 4U)) & 0xFU;
  result.commandedPulseUs = commandedPulseUs;
  result.writeCount = writeCount;
  result.lastChangeMs = lastChangeMs;
  result.lastSource = lastSource;
  return result;
}

const char *servoPulseSourceName(ServoPulseSource source) {
  switch (source) {
    case ServoPulseSource::FixedTest:
      return "FIXED_TEST";
    case ServoPulseSource::TubeTarget:
      return "TUBE_TARGET";
    case ServoPulseSource::Wiggle:
      return "WIGGLE";
    case ServoPulseSource::None:
    default:
      return "NONE";
  }
}
