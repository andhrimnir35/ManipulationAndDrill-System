"""Static/model regression checks for deterministic PA0/TIM5 servo PWM.

These checks deliberately do not claim a physical oscilloscope or servo test.
They prove the register constants, five-state calibration, speed-dependent
state ramp, exact final targets, command isolation, and stop/restart state.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path


SERVO_MIN_US = 800
SERVO_MAX_US = 2200
TIM5_CLOCK_HZ = 84_000_000
TICK_HZ = 1_000_000
PERIOD_US = 20_000
SERVO_STATE_COUNT = 5
RAMP_INTERVAL_MS = 20
STANDARD_MIN_US = 1000
STANDARD_MAX_US = 2000
STANDARD_HALF_TRAVEL_DEG = 45.0
MAX_ABSOLUTE_ANGLE_DEG = 82.5


def angle_for_pulse(pulse: float) -> float:
    if pulse <= STANDARD_MIN_US:
        return -MAX_ABSOLUTE_ANGLE_DEG + (
            (pulse - SERVO_MIN_US)
            * (MAX_ABSOLUTE_ANGLE_DEG - STANDARD_HALF_TRAVEL_DEG)
            / (STANDARD_MIN_US - SERVO_MIN_US)
        )
    if pulse <= STANDARD_MAX_US:
        return -STANDARD_HALF_TRAVEL_DEG + (
            (pulse - STANDARD_MIN_US)
            * (2.0 * STANDARD_HALF_TRAVEL_DEG)
            / (STANDARD_MAX_US - STANDARD_MIN_US)
        )
    return STANDARD_HALF_TRAVEL_DEG + (
        (pulse - STANDARD_MAX_US)
        * (MAX_ABSOLUTE_ANGLE_DEG - STANDARD_HALF_TRAVEL_DEG)
        / (SERVO_MAX_US - STANDARD_MAX_US)
    )


def pulse_for_angle(angle: float) -> float:
    if angle <= -STANDARD_HALF_TRAVEL_DEG:
        return SERVO_MIN_US + (
            (angle + MAX_ABSOLUTE_ANGLE_DEG)
            * (STANDARD_MIN_US - SERVO_MIN_US)
            / (MAX_ABSOLUTE_ANGLE_DEG - STANDARD_HALF_TRAVEL_DEG)
        )
    if angle <= STANDARD_HALF_TRAVEL_DEG:
        return STANDARD_MIN_US + (
            (angle + STANDARD_HALF_TRAVEL_DEG)
            * (STANDARD_MAX_US - STANDARD_MIN_US)
            / (2.0 * STANDARD_HALF_TRAVEL_DEG)
        )
    return STANDARD_MAX_US + (
        (angle - STANDARD_HALF_TRAVEL_DEG)
        * (SERVO_MAX_US - STANDARD_MAX_US)
        / (MAX_ABSOLUTE_ANGLE_DEG - STANDARD_HALF_TRAVEL_DEG)
    )


@dataclass
class DirectPwmModel:
    enabled: bool = False
    pulse_us: int = 1500
    compare_writes: int = 0
    source: str = "NONE"

    def write(self, pulse_us: int, source: str) -> None:
        if not SERVO_MIN_US <= pulse_us <= SERVO_MAX_US:
            raise ValueError("pulse range")
        if not self.enabled or pulse_us != self.pulse_us:
            self.compare_writes += 1
        self.enabled = True
        self.pulse_us = pulse_us
        self.source = source

    def idle(self) -> None:
        """A main-loop pass has no authority to mutate the compare value."""

    def stop(self) -> None:
        self.enabled = False
        self.source = "NONE"


def ramp_to(
    pwm: DirectPwmModel,
    target: int,
    speed_dps: int,
    source: str,
) -> tuple[int, list[int]]:
    if not SERVO_MIN_US <= target <= SERVO_MAX_US:
        raise ValueError("pulse range")
    if not 5 <= speed_dps <= 120:
        raise ValueError("speed")
    ramp_pulse = float(pwm.pulse_us)
    sequence: list[int] = []
    for tick in range(1, 10_000):
        current_angle = angle_for_pulse(ramp_pulse)
        target_angle = angle_for_pulse(float(target))
        distance = target_angle - current_angle
        step = speed_dps * RAMP_INTERVAL_MS / 1000.0
        if abs(distance) <= step:
            ramp_pulse = float(target)
        else:
            ramp_pulse = pulse_for_angle(
                current_angle + (step if distance > 0.0 else -step)
            )
        next_pulse = math.floor(ramp_pulse + 0.5)
        next_pulse = min(SERVO_MAX_US, max(SERVO_MIN_US, next_pulse))
        if next_pulse != pwm.pulse_us:
            pwm.write(next_pulse, source)
            sequence.append(next_pulse)
        if ramp_pulse == float(target):
            return tick * RAMP_INTERVAL_MS, sequence
    raise AssertionError("ramp did not converge")


@dataclass
class TubeModel:
    pwm: DirectPwmModel = field(default_factory=DirectPwmModel)
    pulses: dict[int, int] = field(default_factory=dict)
    invert: bool = False

    def save(self, state: int, pulse_us: int) -> None:
        if state not in range(1, SERVO_STATE_COUNT + 1):
            raise ValueError("state")
        proposed = dict(self.pulses)
        proposed[state] = pulse_us
        ordered = sorted(proposed.items())
        pairs = zip(ordered, ordered[1:])
        if any(
            (a_pulse <= b_pulse if self.invert else a_pulse >= b_pulse)
            for (_, a_pulse), (_, b_pulse) in pairs
        ):
            raise ValueError("calibration order")
        self.pulses = proposed

    def select(self, state: int, speed_dps: int) -> tuple[int, list[int]]:
        if state not in self.pulses:
            raise ValueError("not calibrated")
        return ramp_to(
            self.pwm,
            self.pulses[state],
            speed_dps,
            "TUBE_TARGET",
        )


def run() -> None:
    root = Path(__file__).parents[1]
    main = (root / "src" / "main.cpp").read_text(encoding="utf-8")
    driver = (root / "lib" / "servo_pwm" / "servo_pwm.cpp").read_text(
        encoding="utf-8"
    )
    minimal = (root / "src" / "servo_minimal_test.cpp").read_text(
        encoding="utf-8"
    )
    gui = (root / "tools" / "f401_test_gui.py").read_text(encoding="utf-8")
    web_html = gui.split('WEB_HTML = r"""', 1)[1].split(
        '"""\n\n\nclass WebGuiState', 1
    )[0]
    platformio = (root / "platformio.ini").read_text(encoding="utf-8")

    # 1: F401 APB1 timer clock / 1 MHz tick / 50 Hz frame math.
    assert TIM5_CLOCK_HZ // TICK_HZ - 1 == 83
    assert PERIOD_US - 1 == 19_999

    # 2-4: the low-level/minimal-test targets map directly to CCR1.
    pwm = DirectPwmModel()
    for expected_writes, pulse in enumerate((1000, 1500, 2000), start=1):
        pwm.write(pulse, "FIXED_TEST")
        assert pwm.enabled and pwm.pulse_us == pulse
        assert pwm.compare_writes == expected_writes

    # 5: an idle loop cannot change the target or create writes.
    before = (pwm.pulse_us, pwm.compare_writes, pwm.source)
    for _ in range(10_000):
        pwm.idle()
    assert (pwm.pulse_us, pwm.compare_writes, pwm.source) == before

    # 6: repeating an identical user target does not write CCR1 again.
    pwm.write(2000, "FIXED_TEST")
    assert pwm.compare_writes == 3

    # 7: out-of-range targets are rejected without altering state.
    try:
        pwm.write(799, "FIXED_TEST")
    except ValueError:
        pass
    else:
        raise AssertionError("799 us must be rejected")
    assert pwm.pulse_us == 2000 and pwm.compare_writes == 3

    # 8-9: stop disables PWM; a later command restarts at the exact target.
    pwm.stop()
    assert not pwm.enabled and pwm.source == "NONE"
    pwm.write(1500, "FIXED_TEST")
    assert pwm.enabled and pwm.pulse_us == 1500 and pwm.compare_writes == 4

    # 10: power-up/reset model begins with PWM disabled.
    reset_pwm = DirectPwmModel()
    assert not reset_pwm.enabled

    # 11-14: five calibrated states remain ordered; labels are independently
    # mirrored so state 1 is Final lid and state 5 is Tube 1.
    tubes = TubeModel()
    safe_five_state_pulses = (800, 1150, 1500, 1850, 2200)
    for state, pulse in enumerate(safe_five_state_pulses, start=1):
        tubes.save(state, pulse)
    assert len(tubes.pulses) == SERVO_STATE_COUNT
    tubes.pwm = DirectPwmModel(enabled=True, pulse_us=800)
    elapsed_ms, sequence = tubes.select(5, 30)
    assert elapsed_ms > 0
    assert sequence == sorted(sequence)
    assert tubes.pwm.pulse_us == 2200
    assert all(SERVO_MIN_US <= pulse <= SERVO_MAX_US for pulse in sequence)
    sequential = TubeModel(
        pwm=DirectPwmModel(enabled=True, pulse_us=1500),
        pulses=dict(tubes.pulses),
    )
    for state, expected_pulse in enumerate(safe_five_state_pulses, start=1):
        _, state_sequence = sequential.select(state, 30)
        assert sequential.pwm.pulse_us == expected_pulse
        assert all(
            SERVO_MIN_US <= pulse <= SERVO_MAX_US
            for pulse in state_sequence
        )
    try:
        TubeModel().select(2, 30)
    except ValueError:
        pass
    else:
        raise AssertionError("uncalibrated tube must be rejected")
    try:
        tubes.save(2, 1700)
    except ValueError:
        pass
    else:
        raise AssertionError("crossed tube calibration must be rejected")

    # 15: configured speed changes duration, not the exact final pulse.
    slow = TubeModel(
        pwm=DirectPwmModel(enabled=True, pulse_us=800),
        pulses=dict(tubes.pulses),
    )
    fast = TubeModel(
        pwm=DirectPwmModel(enabled=True, pulse_us=800),
        pulses=dict(tubes.pulses),
    )
    slow_ms, slow_sequence = slow.select(5, 5)
    fast_ms, fast_sequence = fast.select(5, 120)
    assert slow_ms > fast_ms
    assert len(slow_sequence) > len(fast_sequence)
    assert slow.pwm.pulse_us == fast.pwm.pulse_us == 2200

    # Production custom-pulse moves use the same speed ramp. One-click manual
    # steps change exactly the requested pulse amount without a repeat loop.
    fixed_slow = DirectPwmModel(enabled=True, pulse_us=1500)
    fixed_fast = DirectPwmModel(enabled=True, pulse_us=1500)
    fixed_slow_ms, fixed_slow_sequence = ramp_to(
        fixed_slow, 1000, 5, "FIXED_TEST"
    )
    fixed_fast_ms, fixed_fast_sequence = ramp_to(
        fixed_fast, 1000, 120, "FIXED_TEST"
    )
    assert fixed_slow_ms > fixed_fast_ms
    assert fixed_slow_sequence[-1] == fixed_fast_sequence[-1] == 1000
    step_pwm = DirectPwmModel(enabled=True, pulse_us=1500)
    _, backward_sequence = ramp_to(step_pwm, 1490, 30, "FIXED_TEST")
    assert backward_sequence[-1] == 1490
    _, forward_sequence = ramp_to(step_pwm, 1500, 30, "FIXED_TEST")
    assert forward_sequence[-1] == 1500

    # 16: PA0 is configured as AF2, push-pull, with no pull resistor.
    assert "GPIOA->MODER" in driver
    assert "(2UL << (SERVO_GPIO_PIN_INDEX * 2U))" in driver
    assert "SERVO_GPIO_AF << (SERVO_GPIO_PIN_INDEX * 4U)" in driver
    assert "GPIOA->OTYPER &= ~(1UL << SERVO_GPIO_PIN_INDEX);" in driver
    assert "GPIOA->PUPDR &= ~(3UL << (SERVO_GPIO_PIN_INDEX * 2U));" in driver

    # 17: TIM5 is configured for 1-us ticks, a 20-ms frame, and direct CCR1.
    for fragment in (
        "TIM5->PSC = prescaler - 1U;",
        "TIM5->ARR = SERVO_PWM_PERIOD_US - 1U;",
        "TIM5->CCR1 = pulseUs;",
        "(SERVO_PWM_MODE_1 << TIM_CCMR1_OC1M_Pos)",
        "TIM5->CCER = TIM_CCER_CC1E;",
        "TIM5->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;",
    ):
        assert fragment in driver, fragment

    # 18: servo PWM has no timer interrupt/DMA writer.
    assert "TIM5->DIER = 0;" in driver
    assert "attachInterrupt" not in driver
    assert "TIM5_IRQHandler" not in driver

    # 19: stop turns TIM5 off and drives PA0 low as a normal output.
    for fragment in (
        "TIM5->CCER &= ~TIM_CCER_CC1E;",
        "TIM5->CR1 &= ~TIM_CR1_CEN;",
        "GPIOA->BSRR = 1UL << (SERVO_GPIO_PIN_INDEX + 16U);",
    ):
        assert fragment in driver, fragment

    # 20: production commands keep source attribution isolated.
    for fragment in (
        "ServoPulseSource::FixedTest",
        "ServoPulseSource::TubeTarget",
        '"SERVO_MANUAL_REMOVED_USE_SERVO_PULSE"',
        '"SERVO_GAMEPAD_CONTROL_DISABLED"',
        "printServoTimerDiagnostics();",
    ):
        assert fragment in main, fragment
    assert "applyServoManualAxisValue" not in main
    assert "applyServoManualJog" not in main
    assert "servoTimer->" not in main

    # 21: normal state selection starts a non-blocking ramp from the current
    # pulse; serviceServo owns the incremental TubeTarget writes.
    state_move = main[
        main.index("bool startServoStateMove")
        : main.index("bool startServoWiggle")
    ]
    assert "ServoPulseSource::TubeTarget" in state_move
    assert "ServoMotion::StateMove" in state_move
    assert "targetPulse, ServoPulseSource::TubeTarget" not in state_move
    service = main[
        main.index("void serviceServo()")
        : main.index("Direction applyInvert")
    ]
    assert "serviceServoRamp(now, ServoPulseSource::TubeTarget)" in service
    assert "servoRuntime.currentState = servoRuntime.targetState;" in service
    assert "SERVO_RAMP_INTERVAL_MS = 20" in main
    assert "SETTINGS_VERSION = 13" in main
    assert "PersistentSettingsV12" in main
    assert "SERVO_STATE_MAX = 5" in main
    assert "ServoMotion::FixedMove" in main
    assert "startServoFixedMove" in main
    assert "serviceServoRamp(now, ServoPulseSource::FixedTest)" in service
    assert 'strcmp(tokens[1], "forward")' in main
    assert 'strcmp(tokens[1], "backward")' in main
    assert "DEFAULT_SERVO_MANUAL_STEP_US = 10" in main

    # 22: GUI exposes all five named states and retains explicit fixed tests.
    for pulse in (1000, 1500, 2000):
        assert f"servo pulse {pulse}" in web_html
    assert "servo pulse off" in web_html and "servo diag" in web_html
    assert "servo jog" not in web_html
    assert "startServoManualButton" not in web_html
    assert "servoManualButtonTimer" not in web_html
    assert "servo 5" in web_html
    assert "Final lid" in web_html
    assert """send('servo 1')">Final lid""" in web_html
    assert """send('servo 2')">Tube 4""" in web_html
    assert """send('servo 3')">Tube 3""" in web_html
    assert """send('servo 4')">Tube 2""" in web_html
    assert """send('servo 5')">Tube 1""" in web_html
    assert "set servospeed" in web_html
    assert "speed-controlled ramp" in web_html
    assert "servo forward '+v('servoStep')" in web_html
    assert "servo backward '+v('servoStep')" in web_html
    assert "onmousedown" not in web_html

    # 23: an independently buildable minimal environment contains only the
    # three fixed pulse choices, off, diagnostics and safety stop.
    assert "[env:blackpill_f401ce_servo_minimal]" in platformio
    for token in (
        "pulseUs != 1000",
        "pulseUs != 1500",
        "pulseUs != 2000",
        '"off"',
        "servo diag;",
    ):
        assert token in minimal, token
    assert "SERVO_MINIMAL_TEST" in minimal
    assert "joy " not in minimal and "servowiggle" not in minimal

    print(
        "deterministic five-state servo PWM: 23 scenarios ok "
        f"(5 dps={slow_ms} ms, 120 dps={fast_ms} ms)"
    )


if __name__ == "__main__":
    run()
