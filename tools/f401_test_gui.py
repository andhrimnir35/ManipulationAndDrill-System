# Requires Python 3. Run with the interpreter configured in .vscode/settings.json.
"""
Bench test GUI for the STM32F401 manipulation/drill firmware.

This tool does not modify or depend on the firmware source. It only sends the
existing text commands over the F401 UART link and prints every reply.

Dependencies:
  python -m pip install pyserial

The desktop Gamepad ARM tab uses pygame. When pygame is not installed in the
Tkinter interpreter, the GUI reuses the pygame installation from PlatformIO's
Python environment in an isolated helper process.

Typical use:
  python tools/f401_test_gui.py --port COM7

If this Python has no tkinter, the tool automatically starts a browser GUI.
You can force that mode with:
  python tools/f401_test_gui.py --web --port COM7 --auto-connect

Dry-run UI test without hardware:   
  python tools/f401_test_gui.py --dry-run
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse
import webbrowser

from python_deps import ensure_pyserial_available

try:
    import tkinter as tk
    from tkinter import messagebox, scrolledtext, ttk
except ImportError:
    tk = None  # type: ignore[assignment]
    messagebox = None  # type: ignore[assignment]
    scrolledtext = None  # type: ignore[assignment]
    ttk = None  # type: ignore[assignment]
from typing import Callable, Deque, Dict, Iterable, List, Optional


ensure_pyserial_available()


DEFAULT_BAUD = 115200
DEFAULT_WEB_HOST = "127.0.0.1"
DEFAULT_WEB_PORT = 8765
HEARTBEAT_INTERVAL_SECONDS = 0.5
MAX_SERIAL_LINE_BYTES = 4096
SERIAL_READ_CHUNK_BYTES = 4096
MAX_DESKTOP_LOG_LINES = 1500
DESKTOP_LOG_LINES_AFTER_TRIM = 1000
SERVO_MIN_US = 800
SERVO_MAX_US = 2200
SERVO_STANDARD_MIN_US = 1000
SERVO_STANDARD_MAX_US = 2000
SERVO_STANDARD_HALF_TRAVEL_DEG = 45.0
SERVO_MAX_ABSOLUTE_ANGLE_DEG = 82.5
SERVO_STATE_COUNT = 5
AXIS_NAMES = [
    "J1 Base",
    "J2 Shoulder",
    "J3 Elbow",
    "J4 Pitch",
    "J5 Twist",
    "J6 Gripper",
]
POSITION_AXES = (1, 2, 3)
STOP_MODES = ("coast", "brake", "hold", "hybrid")
BREAKPOINT_MODES = ("OFF", "WALL", "BLOCK_INCREASE", "BLOCK_DECREASE")
STATUS_FIELDS = ("PWM", "DIR", "EN", "ANGLE", "VEL_DPS")
SERVO_STATUS_FIELDS = (
    "SERVO_INIT",
    "SERVO_STATE",
    "SERVO_TARGET",
    "SERVO_MOVING",
    "SERVOANGLE",
    "SERVOWIGGLEANGLE",
    "SERVO_INVERT",
    "SERVOSPEED",
    "SERVO_PWM",
    "SERVO_PULSE",
    "SERVO_TARGETPULSE",
    "SERVO_SAFETY",
)
SERVO_EXTENDED_FIELDS = (
    "SERVO_REFERENCE_VALID",
    "SERVO_CURRENT_STATE",
    "SERVO_TARGET_STATE",
    "SERVO_CURRENT_PULSE",
    "SERVO_REFERENCE_PULSE",
    "SERVO_TARGET_PULSE",
    "SERVO_ERROR",
    "SERVO_MANUAL_VALUE",
    "SERVO_MANUAL_ENABLED",
    "SERVO_MANUAL_ACTIVE",
    "SERVO_MANUAL_NEUTRAL",
    "SERVO_CONFIGURED_SPEED",
    "SERVO_ACTIVE_SPEED",
    "SERVO_ANGLE_STEP",
    "SERVO_MANUAL_INVERT",
    "SERVO_MANUAL_PWM",
    "SERVO_CURRENT_ANGLE",
    "SERVO_TARGET_ANGLE",
    "SERVO_REFERENCE_ANGLE",
    "SERVO_CAL_MASK",
    "SERVO_CAL_P1",
    "SERVO_CAL_P2",
    "SERVO_CAL_P3",
    "SERVO_CAL_P4",
    "SERVO_CAL_P5",
)
SERVO_SAFETY_STATES = (
    "BOOT",
    "READY",
    "STOP",
    "SAFE_MODE",
    "LINK_LOSS",
    "GAMEPAD_DISCONNECT",
    "CONFIG_CHANGED",
    "FACTORY_RESET",
    "PULSE_RANGE",
    "ESTOP",
    "MANUAL_CONTROL",
)
DRILL_LED_STATUS_FIELDS = tuple(f"DRILL_LED{index}" for index in range(1, 5))
BRAKEPOINT_TUNING_FIELDS = (
    "BPREACT_MS",
    "BPDECEL_DPS2",
    "BPLEAD_DEG",
    "BPCLEAR_DEG",
    "BPRECPWM",
    "BPRECTIME_MS",
)
BRAKEPOINT_COMMAND_TO_FIELD = {
    "bpreact": "BPREACT_MS",
    "bpdecel": "BPDECEL_DPS2",
    "bplead": "BPLEAD_DEG",
    "bpclear": "BPCLEAR_DEG",
    "bprecpwm": "BPRECPWM",
    "bprectime": "BPRECTIME_MS",
}
ELEVATOR_SENDPWM_FIELDS = ("SENDPWM_BTS4", "SENDPWM_BTS5")
BRAKEPOINT_TUNING_DEFAULTS = {
    1: {
        "BPREACT_MS": "40",
        "BPDECEL_DPS2": "500",
        "BPLEAD_DEG": "0.60",
        "BPCLEAR_DEG": "0.80",
        "BPRECPWM": "70",
        "BPRECTIME_MS": "2500",
    },
    2: {
        "BPREACT_MS": "40",
        "BPDECEL_DPS2": "350",
        "BPLEAD_DEG": "0.80",
        "BPCLEAR_DEG": "1.00",
        "BPRECPWM": "90",
        "BPRECTIME_MS": "3500",
    },
    3: {
        "BPREACT_MS": "40",
        "BPDECEL_DPS2": "650",
        "BPLEAD_DEG": "0.60",
        "BPCLEAR_DEG": "0.80",
        "BPRECPWM": "65",
        "BPRECTIME_MS": "2500",
    },
}
GAMEPAD_AXIS_DEFAULTS = (
    ("Base", 1, 0, False),
    ("Shoulder", 2, 1, True),
    ("Elbow", 3, 3, True),
    ("Twist", 5, 2, False),
)
GAMEPAD_BUTTON_DEFAULTS = (
    ("D-pad up", 12, 1000),
    ("D-pad down", 13, 1001),
    ("D-pad left", 14, 1002),
    ("D-pad right", 15, 1003),
)


def servo_angle_for_pulse(pulse: float) -> float:
    if pulse <= SERVO_STANDARD_MIN_US:
        return -SERVO_MAX_ABSOLUTE_ANGLE_DEG + (
            (pulse - SERVO_MIN_US)
            * (SERVO_MAX_ABSOLUTE_ANGLE_DEG - SERVO_STANDARD_HALF_TRAVEL_DEG)
            / (SERVO_STANDARD_MIN_US - SERVO_MIN_US)
        )
    if pulse <= SERVO_STANDARD_MAX_US:
        return -SERVO_STANDARD_HALF_TRAVEL_DEG + (
            (pulse - SERVO_STANDARD_MIN_US)
            * (2 * SERVO_STANDARD_HALF_TRAVEL_DEG)
            / (SERVO_STANDARD_MAX_US - SERVO_STANDARD_MIN_US)
        )
    return SERVO_STANDARD_HALF_TRAVEL_DEG + (
        (pulse - SERVO_STANDARD_MAX_US)
        * (SERVO_MAX_ABSOLUTE_ANGLE_DEG - SERVO_STANDARD_HALF_TRAVEL_DEG)
        / (SERVO_MAX_US - SERVO_STANDARD_MAX_US)
    )


def servo_pulse_for_angle(angle: float) -> float:
    if angle <= -SERVO_STANDARD_HALF_TRAVEL_DEG:
        return SERVO_MIN_US + (
            (angle + SERVO_MAX_ABSOLUTE_ANGLE_DEG)
            * (SERVO_STANDARD_MIN_US - SERVO_MIN_US)
            / (SERVO_MAX_ABSOLUTE_ANGLE_DEG - SERVO_STANDARD_HALF_TRAVEL_DEG)
        )
    if angle <= SERVO_STANDARD_HALF_TRAVEL_DEG:
        return SERVO_STANDARD_MIN_US + (
            (angle + SERVO_STANDARD_HALF_TRAVEL_DEG)
            * (SERVO_STANDARD_MAX_US - SERVO_STANDARD_MIN_US)
            / (2 * SERVO_STANDARD_HALF_TRAVEL_DEG)
        )
    return SERVO_STANDARD_MAX_US + (
        (angle - SERVO_STANDARD_HALF_TRAVEL_DEG)
        * (SERVO_MAX_US - SERVO_STANDARD_MAX_US)
        / (SERVO_MAX_ABSOLUTE_ANGLE_DEG - SERVO_STANDARD_HALF_TRAVEL_DEG)
    )


def servo_tube1_reference_bounds(angle_deg: int, invert: bool) -> tuple[int, int]:
    """Return the Tube 1 pulse range that keeps every configured state safe."""
    valid: List[int] = []
    for reference_pulse in range(SERVO_MIN_US, SERVO_MAX_US + 1):
        reference_angle = servo_angle_for_pulse(reference_pulse)
        target_angles = [
            reference_angle + state * angle_deg * (-1 if invert else 1)
            for state in range(SERVO_STATE_COUNT)
        ]
        if all(
            -SERVO_MAX_ABSOLUTE_ANGLE_DEG
            <= target_angle
            <= SERVO_MAX_ABSOLUTE_ANGLE_DEG
            for target_angle in target_angles
        ):
            valid.append(reference_pulse)
    if not valid:
        raise ValueError("configured tube spacing has no safe Tube 1 pulse")
    return valid[0], valid[-1]


def servo_state_label(state: object) -> str:
    text = str(state)
    return {
        "1": "Final lid",
        "2": "Tube 4",
        "3": "Tube 3",
        "4": "Tube 2",
        "5": "Tube 1",
    }.get(text, text)


def blank_status_axes() -> List[Dict[str, str]]:
    return [{field: "-" for field in STATUS_FIELDS} for _ in AXIS_NAMES]


def blank_status_snapshot() -> Dict[str, object]:
    return {
        "mode": "-",
        "stopmode": "-",
        "axes": blank_status_axes(),
        "servo": {
            field: "-"
            for field in (*SERVO_STATUS_FIELDS, *SERVO_EXTENDED_FIELDS)
        },
        "drill_leds": ["-"] * 4,
    }


def parse_status_line(line: str) -> Optional[Dict[str, object]]:
    if not line.startswith("MODE:"):
        return None
    snapshot = blank_status_snapshot()
    axes = snapshot["axes"]
    servo = snapshot["servo"]
    drill_leds = snapshot["drill_leds"]
    assert isinstance(axes, list)
    assert isinstance(servo, dict)
    assert isinstance(drill_leds, list)
    for token in line.split():
        key, separator, value = token.partition(":")
        if not separator:
            continue
        key = key.upper()
        if key == "MODE":
            snapshot["mode"] = value
            continue
        if key == "STOPMODE":
            snapshot["stopmode"] = value
            continue
        if key in SERVO_STATUS_FIELDS:
            servo[key] = value
            continue
        if key in DRILL_LED_STATUS_FIELDS:
            drill_leds[int(key[-1]) - 1] = value
            continue
        if len(key) < 4 or key[0] != "J" or not key[1].isdigit() or key[2] != "_":
            continue
        axis_index = int(key[1]) - 1
        field = key[3:]
        if 0 <= axis_index < len(axes) and field in STATUS_FIELDS:
            axes[axis_index][field] = value
    if snapshot["mode"] not in ("safe", "arm", "armtest", "drill"):
        return None
    if snapshot["stopmode"] not in STOP_MODES:
        return None
    for axis in axes:
        if any(axis[field] == "-" for field in STATUS_FIELDS):
            return None
        try:
            pwm = int(axis["PWM"])
            enabled = int(axis["EN"])
        except ValueError:
            return None
        if not 0 <= pwm <= 255 or enabled not in (0, 1):
            return None
        if axis["DIR"] not in ("stop", "forward", "backward"):
            return None
        for field in ("ANGLE", "VEL_DPS"):
            if axis[field] == "NA":
                continue
            try:
                value = float(axis[field])
            except ValueError:
                return None
            if not math.isfinite(value):
                return None
    return snapshot


def parse_servo_status_line(line: str) -> Optional[Dict[str, str]]:
    if not line.startswith("SERVO "):
        return None
    assignments: Dict[str, str] = {}
    for token in line.split()[1:]:
        key, separator, value = token.partition(":")
        if separator and value:
            assignments[key] = value
    expected = (
        "INIT",
        "STATE",
        "TARGET",
        "MOVING",
        "ANGLE",
        "WIGGLE",
        "INVERT",
        "SPEED",
        "PWM",
        "PULSE",
        "TARGETPULSE",
        "SAFETY",
    )
    if any(field not in assignments for field in expected):
        return None
    try:
        initialized = int(assignments["INIT"])
        moving = int(assignments["MOVING"])
        angle = int(assignments["ANGLE"])
        wiggle = int(assignments["WIGGLE"])
        invert = int(assignments["INVERT"])
        speed = int(assignments["SPEED"])
        pwm_active = int(assignments["PWM"])
        pulse = int(assignments["PULSE"])
        target_pulse = int(assignments["TARGETPULSE"])
    except ValueError:
        return None
    if (
        initialized not in (0, 1)
        or moving not in (0, 1)
        or invert not in (0, 1)
        or pwm_active not in (0, 1)
        or not 1 <= angle <= 55
        or not 1 <= wiggle <= 30
        or not 5 <= speed <= 120
        or not 800 <= pulse <= 2200
        or not 800 <= target_pulse <= 2200
        or assignments["SAFETY"] not in SERVO_SAFETY_STATES
    ):
        return None
    if initialized == 0:
        if assignments["STATE"] != "NA" or assignments["TARGET"] != "NA":
            return None
    else:
        for state_field in ("STATE", "TARGET"):
            if assignments[state_field] == "NA":
                continue
            try:
                state = int(assignments[state_field])
            except ValueError:
                return None
            if not 1 <= state <= SERVO_STATE_COUNT:
                return None
    return {
        "SERVO_INIT": assignments["INIT"],
        "SERVO_STATE": assignments["STATE"],
        "SERVO_TARGET": assignments["TARGET"],
        "SERVO_MOVING": assignments["MOVING"],
        "SERVOANGLE": assignments["ANGLE"],
        "SERVOWIGGLEANGLE": assignments["WIGGLE"],
        "SERVO_INVERT": assignments["INVERT"],
        "SERVOSPEED": assignments["SPEED"],
        "SERVO_PWM": assignments["PWM"],
        "SERVO_PULSE": assignments["PULSE"],
        "SERVO_TARGETPULSE": assignments["TARGETPULSE"],
        "SERVO_SAFETY": assignments["SAFETY"],
    }


def parse_servo_extended_line(line: str) -> Optional[Dict[str, str]]:
    assignments: Dict[str, str] = {}
    if line.startswith("SERVOREFX "):
        for token in line.split()[1:]:
            key, separator, value = token.partition(":")
            if separator and value:
                assignments[key] = value
        expected = (
            "VALID",
            "CURRENTSTATE",
            "TARGETSTATE",
            "CURRENTPULSE",
            "REFERENCEPULSE",
            "TARGETPULSE",
            "ERROR",
        )
        if any(field not in assignments for field in expected):
            return None
        try:
            valid = int(assignments["VALID"])
            pulses = [
                int(assignments["CURRENTPULSE"]),
                int(assignments["REFERENCEPULSE"]),
                int(assignments["TARGETPULSE"]),
            ]
        except ValueError:
            return None
        if valid not in (0, 1) or any(not 800 <= pulse <= 2200 for pulse in pulses):
            return None
        for state_field in ("CURRENTSTATE", "TARGETSTATE"):
            if assignments[state_field] == "NA":
                continue
            try:
                state = int(assignments[state_field])
            except ValueError:
                return None
            if not 1 <= state <= SERVO_STATE_COUNT:
                return None
        return {
            "SERVO_REFERENCE_VALID": assignments["VALID"],
            "SERVO_CURRENT_STATE": assignments["CURRENTSTATE"],
            "SERVO_TARGET_STATE": assignments["TARGETSTATE"],
            "SERVO_CURRENT_PULSE": assignments["CURRENTPULSE"],
            "SERVO_REFERENCE_PULSE": assignments["REFERENCEPULSE"],
            "SERVO_TARGET_PULSE": assignments["TARGETPULSE"],
            "SERVO_ERROR": assignments["ERROR"],
        }

    if line.startswith("SERVOMANUAL "):
        for token in line.split()[1:]:
            key, separator, value = token.partition(":")
            if separator and value:
                assignments[key] = value
        expected = (
            "VALUE",
            "ENABLED",
            "ACTIVE",
            "NEUTRAL",
            "CONFIGSPEED",
            "ACTIVESPEED",
            "ANGLESTEP",
            "INVERT",
            "PWM",
        )
        if any(field not in assignments for field in expected):
            return None
        try:
            value = int(assignments["VALUE"])
            enabled = int(assignments["ENABLED"])
            active = int(assignments["ACTIVE"])
            neutral = int(assignments["NEUTRAL"])
            configured_speed = int(assignments["CONFIGSPEED"])
            active_speed = int(assignments["ACTIVESPEED"])
            angle_step = int(assignments["ANGLESTEP"])
            invert = int(assignments["INVERT"])
            pwm = int(assignments["PWM"])
        except ValueError:
            return None
        if (
            not -1000 <= value <= 1000
            or enabled not in (0, 1)
            or active not in (0, 1)
            or neutral not in (0, 1)
            or not 5 <= configured_speed <= 120
            or not 0 <= active_speed <= 120
            or not 1 <= angle_step <= 55
            or invert not in (0, 1)
            or pwm not in (0, 1)
        ):
            return None
        return {
            "SERVO_MANUAL_VALUE": assignments["VALUE"],
            "SERVO_MANUAL_ENABLED": assignments["ENABLED"],
            "SERVO_MANUAL_ACTIVE": assignments["ACTIVE"],
            "SERVO_MANUAL_NEUTRAL": assignments["NEUTRAL"],
            "SERVO_CONFIGURED_SPEED": assignments["CONFIGSPEED"],
            "SERVO_ACTIVE_SPEED": assignments["ACTIVESPEED"],
            "SERVO_ANGLE_STEP": assignments["ANGLESTEP"],
            "SERVO_MANUAL_INVERT": assignments["INVERT"],
            "SERVO_MANUAL_PWM": assignments["PWM"],
        }

    if line.startswith("SERVOANGLES "):
        for token in line.split()[1:]:
            key, separator, value = token.partition(":")
            if separator and value:
                assignments[key] = value
        expected = ("VALID", "CURRENT", "TARGET", "REFERENCE")
        if any(field not in assignments for field in expected):
            return None
        try:
            valid = int(assignments["VALID"])
        except ValueError:
            return None
        if valid not in (0, 1):
            return None
        if valid == 0:
            if any(assignments[field] != "NA" for field in expected[1:]):
                return None
        else:
            try:
                angles = [float(assignments[field]) for field in expected[1:]]
            except ValueError:
                return None
            if any(not -180.0 <= angle <= 180.0 for angle in angles):
                return None
        return {
            "SERVO_CURRENT_ANGLE": assignments["CURRENT"],
            "SERVO_TARGET_ANGLE": assignments["TARGET"],
            "SERVO_REFERENCE_ANGLE": assignments["REFERENCE"],
        }

    if line.startswith("SERVOCAL "):
        for token in line.split()[1:]:
            key, separator, value = token.partition(":")
            if separator and value:
                assignments[key] = value
        pulse_fields = tuple(
            f"P{state}" for state in range(1, SERVO_STATE_COUNT + 1)
        )
        expected = ("MASK", *pulse_fields)
        if any(field not in assignments for field in expected):
            return None
        try:
            mask = int(assignments["MASK"])
        except ValueError:
            return None
        if not 0 <= mask <= (1 << SERVO_STATE_COUNT) - 1:
            return None
        for index, field in enumerate(expected[1:]):
            value = assignments[field]
            valid = bool(mask & (1 << index))
            if not valid:
                if value != "NA":
                    return None
                continue
            try:
                pulse = int(value)
            except ValueError:
                return None
            if not SERVO_MIN_US <= pulse <= SERVO_MAX_US:
                return None
        result = {"SERVO_CAL_MASK": assignments["MASK"]}
        result.update(
            {
                f"SERVO_CAL_P{state}": assignments[f"P{state}"]
                for state in range(1, SERVO_STATE_COUNT + 1)
            }
        )
        return result
    return None


def parse_drill_led_status_line(line: str) -> Optional[List[str]]:
    if not line.startswith("DRILLLEDS "):
        return None
    assignments: Dict[str, str] = {}
    for token in line.split()[1:]:
        key, separator, value = token.partition(":")
        if separator and value:
            assignments[key] = value
    values = [assignments.get(f"LED{index}", "-") for index in range(1, 5)]
    if any(value not in ("0", "1") for value in values):
        return None
    return values


def parse_brakepoint_tuning_line(line: str) -> Optional[tuple[int, Dict[str, str]]]:
    """Extract the J1-J3 brakepoint model values printed by `params`."""
    tokens = line.split()
    if not tokens or len(tokens[0]) != 2 or tokens[0][0] != "J" or not tokens[0][1].isdigit():
        return None
    axis = int(tokens[0][1])
    if axis not in BRAKEPOINT_TUNING_DEFAULTS:
        return None
    assignments: Dict[str, str] = {}
    for token in tokens[1:]:
        key, separator, value = token.partition("=")
        if separator and key in BRAKEPOINT_TUNING_FIELDS and value:
            assignments[key] = value
    if any(field not in assignments for field in BRAKEPOINT_TUNING_FIELDS):
        return None
    try:
        reaction_ms = int(assignments["BPREACT_MS"])
        decel_dps2 = int(assignments["BPDECEL_DPS2"])
        lead_deg = float(assignments["BPLEAD_DEG"])
        clearance_deg = float(assignments["BPCLEAR_DEG"])
        recovery_pwm = int(assignments["BPRECPWM"])
        recovery_time_ms = int(assignments["BPRECTIME_MS"])
    except ValueError:
        return None
    numeric_values = (lead_deg, clearance_deg)
    if not all(math.isfinite(value) for value in numeric_values):
        return None
    if not (
        0 <= reaction_ms <= 500
        and 50 <= decel_dps2 <= 20000
        and 0.1 <= lead_deg <= 20.0
        and 0.2 <= clearance_deg <= 20.0
        and 0 <= recovery_pwm <= 255
        and 250 <= recovery_time_ms <= 10000
    ):
        return None
    return axis, assignments


def parse_brakepoint_slot_line(line: str) -> Optional[tuple[int, int, Dict[str, str]]]:
    tokens = line.split()
    if len(tokens) < 5 or tokens[0] != "BP" or len(tokens[1]) != 2:
        return None
    if tokens[1][0] != "J" or not tokens[1][1].isdigit():
        return None
    axis = int(tokens[1][1])
    if axis not in (1, 2, 3):
        return None
    assignments: Dict[str, str] = {}
    for token in tokens[2:]:
        key, separator, value = token.partition("=")
        if separator:
            assignments[key] = value
    try:
        slot = int(assignments["SLOT"])
    except (KeyError, ValueError):
        return None
    mode = assignments.get("MODE", "")
    angle = assignments.get("ANGLE", "")
    if not 1 <= slot <= 4 or mode not in BREAKPOINT_MODES:
        return None
    if mode == "OFF":
        if angle != "NA":
            return None
    else:
        try:
            angle_value = float(angle)
        except ValueError:
            return None
        if not math.isfinite(angle_value) or not 0.0 <= angle_value < 360.0:
            return None
    return axis, slot, {"MODE": mode, "ANGLE": angle}


def parse_elevator_params_line(line: str) -> Optional[Dict[str, str]]:
    """Extract elevator-only BTS4/BTS5 sendpwm values printed by `params`."""
    if not line.startswith("ELV "):
        return None
    assignments: Dict[str, str] = {}
    for token in line.split()[1:]:
        key, separator, value = token.partition("=")
        if separator and key in ELEVATOR_SENDPWM_FIELDS and value:
            assignments[key] = value
    if any(field not in assignments for field in ELEVATOR_SENDPWM_FIELDS):
        return None
    try:
        values = [float(assignments[field]) for field in ELEVATOR_SENDPWM_FIELDS]
    except ValueError:
        return None
    if not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in values):
        return None
    return {
        field: f"{value:.2f}"
        for field, value in zip(ELEVATOR_SENDPWM_FIELDS, values)
    }


@dataclass
class PendingSlider:
    value: int = 0
    sent_value: int = 0
    sent_at: float = 0.0


def clamp_int(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def serial_ports() -> List[str]:
    try:
        from serial.tools import list_ports  # type: ignore
    except ImportError:
        return []
    return [port.device for port in list_ports.comports()]


def scale_gamepad_axis(raw: float, invert: bool, dead: int) -> int:
    value = -raw if invert else raw
    value = max(-1.0, min(1.0, value))
    scaled = int(round(value * 1000.0))
    return 0 if abs(scaled) <= clamp_int(dead, 0, 400) else scaled


def gamepad_subprocess_env() -> Dict[str, str]:
    env = dict(os.environ)
    env["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
    return env


def gamepad_subprocess_flags() -> int:
    return int(getattr(subprocess, "CREATE_NO_WINDOW", 0))


def find_pygame_python() -> Optional[str]:
    candidates = [Path(sys.executable)]
    configured_core = os.environ.get("PLATFORMIO_CORE_DIR")
    core_dir = Path(configured_core).expanduser() if configured_core else Path.home() / ".platformio"
    candidates.extend(
        (
            core_dir / "penv" / "Scripts" / "python.exe",
            core_dir / "penv" / "bin" / "python",
        )
    )

    checked: set[str] = set()
    for candidate in candidates:
        candidate_text = str(candidate)
        normalized = os.path.normcase(os.path.abspath(candidate_text))
        if normalized in checked or not candidate.is_file():
            continue
        checked.add(normalized)
        try:
            result = subprocess.run(
                [candidate_text, "-c", "import pygame"],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5,
                env=gamepad_subprocess_env(),
                creationflags=gamepad_subprocess_flags(),
                check=False,
            )
        except (OSError, subprocess.SubprocessError):
            continue
        if result.returncode == 0:
            return candidate_text
    return None


def run_gamepad_helper(args: argparse.Namespace) -> int:
    try:
        import pygame  # type: ignore
    except ImportError:
        print(json.dumps({"type": "error", "message": "pygame is not installed"}), flush=True)
        return 2

    try:
        pygame.display.init()
        pygame.joystick.init()
        if args.gamepad_helper == "list":
            devices = []
            for index in range(pygame.joystick.get_count()):
                joystick = pygame.joystick.Joystick(index)
                joystick.init()
                devices.append(
                    {
                        "index": index,
                        "name": joystick.get_name(),
                        "guid": joystick.get_guid(),
                        "axes": joystick.get_numaxes(),
                        "buttons": joystick.get_numbuttons(),
                        "hats": joystick.get_numhats(),
                    }
                )
            print(json.dumps({"type": "devices", "devices": devices}), flush=True)
            return 0

        index = int(args.gamepad_index)
        count = pygame.joystick.get_count()
        if not 0 <= index < count:
            print(
                json.dumps({"type": "error", "message": f"gamepad index {index} is no longer available"}),
                flush=True,
            )
            return 2
        joystick = pygame.joystick.Joystick(index)
        joystick.init()
        poll_seconds = clamp_int(int(args.gamepad_helper_poll_ms), 10, 200) / 1000.0
        while True:
            pygame.event.pump()
            snapshot = {
                "type": "state",
                "index": index,
                "name": joystick.get_name(),
                "axes": [joystick.get_axis(axis) for axis in range(joystick.get_numaxes())],
                "buttons": [1 if joystick.get_button(button) else 0 for button in range(joystick.get_numbuttons())],
                "hats": [list(joystick.get_hat(hat)) for hat in range(joystick.get_numhats())],
            }
            print(json.dumps(snapshot, separators=(",", ":")), flush=True)
            time.sleep(poll_seconds)
    except (BrokenPipeError, KeyboardInterrupt):
        return 0
    except Exception as exc:
        try:
            print(json.dumps({"type": "error", "message": str(exc)}), flush=True)
        except BrokenPipeError:
            pass
        return 2
    finally:
        pygame.joystick.quit()
        pygame.display.quit()


class SerialLink:
    def __init__(self, log: Callable[[str], None]) -> None:
        self.log = log
        self.serial = None
        self.dry_run = False
        self._lock = threading.RLock()
        self._rx_buffer = bytearray()
        self._rx_lines: Deque[str] = deque()
        self._discarding_oversize_line = False

    def is_open(self) -> bool:
        with self._lock:
            return self.dry_run or self.serial is not None

    def connect(self, port: str, baud: int, dry_run: bool) -> None:
        with self._lock:
            self._close_locked()
            if dry_run:
                self.dry_run = True
            else:
                if not port:
                    raise RuntimeError("Select a serial port first.")
                try:
                    import serial  # type: ignore
                except ImportError as exc:
                    raise RuntimeError("pyserial missing. Install with: python -m pip install pyserial") from exc
                self.serial = serial.Serial(port, baudrate=baud, timeout=0, write_timeout=0.2)
                time.sleep(0.2)
        if dry_run:
            self.log("LOCAL: dry-run serial link ready")
        else:
            self.log(f"LOCAL: connected to {port} @ {baud}")

    def close(self) -> None:
        with self._lock:
            self._close_locked()

    def _close_locked(self) -> None:
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
            finally:
                self.serial = None
        self.dry_run = False
        self._rx_buffer.clear()
        self._rx_lines.clear()
        self._discarding_oversize_line = False

    def send(self, line: str, *, log_tx: bool = True) -> None:
        line = line.strip()
        if not line:
            return
        if log_tx:
            self.log(f"TX: {line}")
        error: Optional[str] = None
        with self._lock:
            if self.dry_run:
                return
            if self.serial is None:
                error = "LOCAL: not connected"
            else:
                try:
                    self.serial.write((line + "\n").encode("ascii", errors="ignore"))
                except Exception as exc:  # pragma: no cover - depends on serial hardware
                    error = f"LOCAL: serial write failed, connection closed: {exc}"
                    self._close_locked()
        if error is not None:
            self.log(error)

    def read_lines(self, max_lines: int = 100) -> List[str]:
        if max_lines <= 0:
            return []
        error: Optional[str] = None
        with self._lock:
            if self.dry_run or self.serial is None:
                return []
            try:
                waiting = getattr(self.serial, "in_waiting", 0)
                if waiting > 0:
                    raw = self.serial.read(min(int(waiting), SERIAL_READ_CHUNK_BYTES))
                    self._ingest_bytes_locked(raw)
            except Exception as exc:  # pragma: no cover - depends on serial hardware
                error = f"LOCAL: serial read failed, connection closed: {exc}"
                self._close_locked()

            lines: List[str] = []
            while self._rx_lines and len(lines) < max_lines:
                lines.append(self._rx_lines.popleft())
        if error is not None:
            lines.append(error)
        return lines

    def _ingest_bytes_locked(self, raw: bytes) -> None:
        """Keep incomplete serial data buffered until a real line ending arrives."""
        for byte in raw:
            if self._discarding_oversize_line:
                if byte in (10, 13):
                    self._discarding_oversize_line = False
                continue
            if byte in (10, 13):
                if self._rx_buffer:
                    line = self._rx_buffer.decode("utf-8", errors="replace").strip()
                    self._rx_buffer.clear()
                    if line:
                        self._rx_lines.append(line)
                continue
            self._rx_buffer.append(byte)
            if len(self._rx_buffer) > MAX_SERIAL_LINE_BYTES:
                self._rx_buffer.clear()
                self._discarding_oversize_line = True
                self._rx_lines.append(
                    f"LOCAL: discarded serial line longer than {MAX_SERIAL_LINE_BYTES} bytes"
                )


class F401TestGui:
    def __init__(self, root: tk.Tk, args: argparse.Namespace) -> None:
        self.root = root
        self.root.title("F401 Manipulation/Drill Test GUI")
        self.root.geometry("1280x820")
        self.root.minsize(1100, 700)
        self.args = args
        self.link = SerialLink(self.log)
        self._log_line_count = 0

        self.port_var = tk.StringVar(value=args.port or "")
        self.baud_var = tk.IntVar(value=args.baud)
        self.dry_run_var = tk.BooleanVar(value=args.dry_run)
        self.live_slider_var = tk.BooleanVar(value=False)
        self.connected_var = tk.StringVar(value="Disconnected")
        self.last_mode_var = tk.StringVar(value="Mode: unknown")
        self.last_fault_var = tk.StringVar(value="Fault: unknown")
        self.live_status_mode_var = tk.StringVar(value="Mode: -")
        self.live_status_stopmode_var = tk.StringVar(value="Stopmode: -")
        self.live_status_seen_var = tk.StringVar(value="Live status: waiting")
        self.live_status_vars = [
            {field: tk.StringVar(value="-") for field in STATUS_FIELDS}
            for _ in AXIS_NAMES
        ]

        self.axis_pwm_vars = [tk.IntVar(value=80) for _ in range(6)]
        self.axis_slider_vars = [tk.IntVar(value=0) for _ in range(6)]
        self.axis_slider_state = [PendingSlider() for _ in range(6)]

        self.position_angle_vars = [tk.StringVar(value="0") for _ in range(3)]
        self.rotate_angle_vars = [tk.StringVar(value="10") for _ in range(3)]
        self.stopmode_vars = [tk.StringVar(value="hold") for _ in range(3)]
        self.kp_vars = [tk.IntVar(value=1200) for _ in range(3)]
        self.kd_vars = [tk.IntVar(value=250) for _ in range(3)]
        self.minpwm_vars = [tk.IntVar(value=15) for _ in range(3)]
        self.tolerance_vars = [tk.StringVar(value="0.8") for _ in range(3)]
        self.as5600_vars = [tk.IntVar(value=i) for i in range(3)]

        self.tune_axis_var = tk.IntVar(value=1)
        self.tune_invert_var = tk.IntVar(value=0)
        self.tune_maxpwm_var = tk.IntVar(value=255)
        self.tune_default_var = tk.IntVar(value=100)
        self.tune_posgain_var = tk.IntVar(value=1000)
        self.tune_neggain_var = tk.IntVar(value=1000)
        self.tune_dead_var = tk.IntVar(value=50)
        self.tune_keyfwd_var = tk.IntVar(value=-1)
        self.tune_keyback_var = tk.IntVar(value=-1)
        self.tune_bp_slot_mode_vars = [
            tk.StringVar(value="OFF") for _ in range(4)
        ]
        self.tune_bp_slot_angle_vars = [
            tk.StringVar(value="0") for _ in range(4)
        ]
        self.tune_bp_react_var = tk.IntVar(value=40)
        self.tune_bp_decel_var = tk.IntVar(value=500)
        self.tune_bp_lead_var = tk.StringVar(value="0.60")
        self.tune_bp_clear_var = tk.StringVar(value="0.80")
        self.tune_bp_recpwm_var = tk.IntVar(value=70)
        self.tune_bp_rectime_var = tk.IntVar(value=2500)
        self.tune_bp_status_var = tk.StringVar(value="J1 firmware defaults shown; Params loads live values.")
        self.brakepoint_tuning_by_axis = {
            axis: values.copy() for axis, values in BRAKEPOINT_TUNING_DEFAULTS.items()
        }
        self.brakepoint_tuning_live_axes: set[int] = set()
        self.brakepoint_slots_by_axis = {
            axis: [
                {"MODE": "OFF", "ANGLE": "NA"} for _ in range(4)
            ]
            for axis in (1, 2, 3)
        }
        self.tune_axis_var.trace_add("write", self.on_tune_axis_changed)

        self.elv_pwm_var = tk.IntVar(value=80)
        self.drill_pwm_var = tk.IntVar(value=80)
        self.elv_default_var = tk.IntVar(value=100)
        self.elv_max_var = tk.IntVar(value=255)
        self.elv_invert_var = tk.IntVar(value=0)
        self.elv_sendpwm_vars = [
            tk.StringVar(value="1.00"),
            tk.StringVar(value="1.00"),
        ]
        self.drill_default_var = tk.IntVar(value=100)
        self.drill_max_var = tk.IntVar(value=255)
        self.drill_invert_var = tk.IntVar(value=0)
        self.servo_state_display_var = tk.StringVar(value="Servo state: not initialized")
        self.servo_motion_display_var = tk.StringVar(value="Servo: idle")
        self.servo_angle_display_var = tk.StringVar(
            value="Estimated PWM angle: unavailable"
        )
        self.servo_safety_display_var = tk.StringVar(value="PWM: off | safety: BOOT")
        self.servo_reference_display_var = tk.StringVar(value="Reference: invalid")
        self.servo_manual_display_var = tk.StringVar(
            value="State/fixed moves: speed-controlled ramp | step buttons: one click"
        )
        self.servo_calibration_display_var = tk.StringVar(
            value="Servo calibration: none saved"
        )
        self.servo_reference_state_var = tk.IntVar(value=1)
        self.servo_angle_var = tk.IntVar(value=45)
        self.servo_wiggle_angle_var = tk.IntVar(value=5)
        self.servo_invert_var = tk.IntVar(value=0)
        self.servo_speed_var = tk.IntVar(value=30)
        self.servo_pulse_var = tk.IntVar(value=1500)
        self.servo_step_var = tk.IntVar(value=10)
        self.drill_led_state_vars = [
            tk.StringVar(value=f"Drill LED {index}: -")
            for index in range(1, 5)
        ]

        self.joy_axis_var = tk.IntVar(value=1)
        self.joy_value_var = tk.IntVar(value=0)
        self.joy_button_id_var = tk.IntVar(value=1000)

        self.gamepad_device_var = tk.StringVar(value="")
        self.gamepad_status_var = tk.StringVar(value="Gamepad TX: paused")
        self.gamepad_debug_var = tk.StringVar(value="Click Detect gamepads, then select a device.")
        self.gamepad_dead_var = tk.IntVar(value=50)
        self.gamepad_delta_var = tk.IntVar(value=20)
        self.gamepad_poll_var = tk.IntVar(value=20)
        self.gamepad_axis_joint_vars = [tk.IntVar(value=item[1]) for item in GAMEPAD_AXIS_DEFAULTS]
        self.gamepad_axis_id_vars = [tk.IntVar(value=item[2]) for item in GAMEPAD_AXIS_DEFAULTS]
        self.gamepad_axis_invert_vars = [tk.BooleanVar(value=item[3]) for item in GAMEPAD_AXIS_DEFAULTS]
        self.gamepad_button_id_vars = [tk.IntVar(value=item[1]) for item in GAMEPAD_BUTTON_DEFAULTS]
        self.gamepad_joy_id_vars = [tk.IntVar(value=item[2]) for item in GAMEPAD_BUTTON_DEFAULTS]
        self.gamepad_devices: Dict[int, Dict[str, object]] = {}
        self.gamepad_python: Optional[str] = None
        self.gamepad_process: Optional[subprocess.Popen[str]] = None
        self.gamepad_enabled = False
        self.gamepad_detecting = False
        self.gamepad_start_after_detect = False
        self.gamepad_generation = 0
        self.gamepad_messages: queue.Queue[tuple[str, int, object]] = queue.Queue()
        self.gamepad_last_axis: Dict[str, int] = {}
        self.gamepad_last_axis_sent_at: Dict[str, float] = {}
        self.gamepad_last_buttons: Dict[str, int] = {}
        self.gamepad_latest_axes: List[float] = []
        self.stream_channel_var = tk.IntVar(value=0)
        self.stream_interval_var = tk.IntVar(value=250)
        self.as5600_channel_var = tk.IntVar(value=0)

        self._build_ui()
        self.refresh_ports()
        self.root.after(30, self.poll_serial)
        self.root.after(
            int(HEARTBEAT_INTERVAL_SECONDS * 1000), self.service_heartbeat
        )
        self.root.after(50, self.gamepad_tick)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        if args.auto_connect or args.dry_run:
            self.root.after(100, self.connect)

    def _build_ui(self) -> None:
        self._configure_style()
        self._build_top_bar()
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self._build_console_tab(notebook)
        self._build_arm_tab(notebook)
        self._build_position_tab(notebook)
        self._build_tuning_tab(notebook)
        self._build_drill_tab(notebook)
        self._build_joy_tab(notebook)
        self._build_gamepad_tab(notebook)

    def _configure_style(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Danger.TButton", foreground="white", background="#b00020")
        style.map("Danger.TButton", background=[("active", "#d00028")])
        style.configure("Mode.TButton", padding=(10, 5))

    def _build_top_bar(self) -> None:
        frame = ttk.Frame(self.root, padding=8)
        frame.pack(fill="x")

        ttk.Label(frame, text="Port").pack(side="left")
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, width=12)
        self.port_combo.pack(side="left", padx=(4, 6))
        ttk.Button(frame, text="Refresh", command=self.refresh_ports).pack(side="left", padx=(0, 12))

        ttk.Label(frame, text="Baud").pack(side="left")
        ttk.Entry(frame, textvariable=self.baud_var, width=8).pack(side="left", padx=(4, 8))
        ttk.Checkbutton(frame, text="Dry run", variable=self.dry_run_var).pack(side="left", padx=(0, 8))
        ttk.Button(frame, text="Connect", command=self.connect).pack(side="left", padx=(0, 4))
        ttk.Button(frame, text="Disconnect", command=self.disconnect).pack(side="left", padx=(0, 12))

        ttk.Label(frame, textvariable=self.connected_var).pack(side="left", padx=(0, 14))
        ttk.Label(frame, textvariable=self.last_mode_var).pack(side="left", padx=(0, 14))
        ttk.Label(frame, textvariable=self.last_fault_var).pack(side="left", padx=(0, 14))

        ttk.Button(frame, text="STOP ALL", style="Danger.TButton", command=lambda: self.send("stopall")).pack(
            side="right", padx=(8, 0)
        )

    def _build_console_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text="Console")

        quick = ttk.Frame(tab)
        quick.pack(fill="x")
        self._button_row(
            quick,
            [
                ("SAFE", "mode safe"),
                ("ARM", "mode arm confirm"),
                ("ARMTEST", "mode armtest confirm"),
                ("DRILL", "mode drill confirm"),
                ("Stop", "stop"),
                ("StopAll", "stopall"),
            ],
        )
        self._button_row(
            quick,
            [
                ("Help", "help"),
                ("Params", "params"),
                ("Get mode", "get mode"),
                ("Get fault", "get fault"),
                ("Get motors", "get motors"),
                ("Get sensors", "get sensors"),
                ("Get AS5600", "get as5600"),
                ("Stream off", "stream off"),
            ],
        )

        save_frame = ttk.Frame(tab)
        save_frame.pack(fill="x", pady=(6, 6))
        ttk.Button(save_frame, text="Save", command=lambda: self.send("save")).pack(side="left", padx=(0, 4))
        ttk.Button(save_frame, text="Safe + StopAll + Save", command=self.safe_stop_save).pack(side="left", padx=4)
        ttk.Button(save_frame, text="Factory reset", command=self.confirm_factory_reset).pack(side="left", padx=4)
        ttk.Button(save_frame, text="Quit/Exit firmware", command=lambda: self.send("quit")).pack(side="left", padx=4)

        self._build_live_status_panel(tab)

        seq_frame = ttk.LabelFrame(tab, text="seq wrapper")
        seq_frame.pack(fill="x", pady=(0, 8))
        self.seq_id_var = tk.IntVar(value=1)
        self.seq_command_var = tk.StringVar(value="params")
        ttk.Label(seq_frame, text="id").pack(side="left", padx=(6, 2))
        ttk.Spinbox(seq_frame, from_=0, to=999999, textvariable=self.seq_id_var, width=8).pack(side="left")
        ttk.Label(seq_frame, text="command").pack(side="left", padx=(8, 2))
        ttk.Entry(seq_frame, textvariable=self.seq_command_var).pack(side="left", fill="x", expand=True)
        ttk.Button(seq_frame, text="Send seq", command=self.send_seq).pack(side="left", padx=6)

        self.log_text = scrolledtext.ScrolledText(tab, height=24, wrap="word", state="disabled")
        self.log_text.pack(fill="both", expand=True, pady=(0, 6))

        entry_frame = ttk.Frame(tab)
        entry_frame.pack(fill="x")
        self.command_var = tk.StringVar()
        command_entry = ttk.Entry(entry_frame, textvariable=self.command_var)
        command_entry.pack(side="left", fill="x", expand=True)
        command_entry.bind("<Return>", lambda _event: self.send_manual())
        ttk.Button(entry_frame, text="Send", command=self.send_manual).pack(side="left", padx=(6, 0))
        ttk.Button(entry_frame, text="Clear log", command=self.clear_log).pack(side="left", padx=(6, 0))

    def _build_live_status_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Live F401 status")
        frame.pack(fill="x", pady=(0, 8))

        summary = ttk.Frame(frame)
        summary.pack(fill="x", padx=6, pady=(6, 2))
        ttk.Label(summary, textvariable=self.live_status_mode_var).pack(side="left", padx=(0, 16))
        ttk.Label(summary, textvariable=self.live_status_stopmode_var).pack(side="left", padx=(0, 16))
        ttk.Label(summary, textvariable=self.live_status_seen_var).pack(side="left")

        table = ttk.Frame(frame)
        table.pack(fill="x", padx=6, pady=(2, 6))
        headers = ("Joint", "PWM", "DIR", "EN", "Angle", "Vel dps")
        for column, text in enumerate(headers):
            ttk.Label(table, text=text).grid(row=0, column=column, sticky="w", padx=4, pady=2)
        for row, name in enumerate(AXIS_NAMES, start=1):
            ttk.Label(table, text=name).grid(row=row, column=0, sticky="w", padx=4, pady=2)
            for column, field in enumerate(STATUS_FIELDS, start=1):
                ttk.Label(table, textvariable=self.live_status_vars[row - 1][field], width=12).grid(
                    row=row,
                    column=column,
                    sticky="w",
                    padx=4,
                    pady=2,
                )
        table.columnconfigure(0, weight=1)

    def _build_arm_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text="ARM Drive")

        top = ttk.Frame(tab)
        top.pack(fill="x")
        ttk.Button(top, text="Mode ARM", command=lambda: self.send("mode arm confirm")).pack(side="left", padx=(0, 4))
        ttk.Button(top, text="Mode ARMTEST", command=lambda: self.send("mode armtest confirm")).pack(
            side="left", padx=4
        )
        ttk.Button(top, text="Stop", command=lambda: self.send("stop")).pack(side="left", padx=4)
        ttk.Button(top, text="StopAll", style="Danger.TButton", command=lambda: self.send("stopall")).pack(
            side="left", padx=4
        )
        ttk.Checkbutton(top, text="Live axis sliders", variable=self.live_slider_var).pack(side="left", padx=12)

        grid = ttk.Frame(tab)
        grid.pack(fill="both", expand=True, pady=(8, 0))
        headers = ("Axis", "PWM", "Start", "Analog axis -1000..1000", "Send")
        for col, text in enumerate(headers):
            ttk.Label(grid, text=text).grid(row=0, column=col, sticky="w", padx=4, pady=4)
        grid.columnconfigure(3, weight=1)

        for i, name in enumerate(AXIS_NAMES, start=1):
            row = i
            ttk.Label(grid, text=name, width=13).grid(row=row, column=0, sticky="w", padx=4, pady=4)
            ttk.Spinbox(
                grid,
                from_=0,
                to=255,
                textvariable=self.axis_pwm_vars[i - 1],
                width=6,
            ).grid(row=row, column=1, sticky="w", padx=4)
            start_frame = ttk.Frame(grid)
            start_frame.grid(row=row, column=2, sticky="w", padx=4)
            ttk.Button(start_frame, text="Fwd", command=lambda axis=i: self.drive_axis(axis, "forward")).pack(
                side="left", padx=(0, 2)
            )
            ttk.Button(start_frame, text="Back", command=lambda axis=i: self.drive_axis(axis, "backward")).pack(
                side="left", padx=2
            )
            ttk.Button(start_frame, text="Stop", command=lambda axis=i: self.send(f"stop {axis}")).pack(
                side="left", padx=2
            )
            scale = ttk.Scale(
                grid,
                from_=-1000,
                to=1000,
                orient="horizontal",
                variable=self.axis_slider_vars[i - 1],
                command=lambda value, axis=i: self.on_axis_slider(axis, value),
            )
            scale.grid(row=row, column=3, sticky="ew", padx=4)
            scale.bind("<ButtonRelease-1>", lambda _event, axis=i: self.send_axis_slider(axis))
            send_frame = ttk.Frame(grid)
            send_frame.grid(row=row, column=4, sticky="w", padx=4)
            ttk.Button(send_frame, text="Send", command=lambda axis=i: self.send_axis_slider(axis)).pack(
                side="left", padx=(0, 2)
            )
            ttk.Button(send_frame, text="0+Stop", command=lambda axis=i: self.zero_axis_slider(axis)).pack(
                side="left", padx=2
            )

    def _build_position_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text="Position/PID")

        sensor_frame = ttk.LabelFrame(tab, text="Sensors and stream")
        sensor_frame.pack(fill="x")
        ttk.Button(sensor_frame, text="Get sensors", command=lambda: self.send("get sensors")).pack(
            side="left", padx=4, pady=6
        )
        ttk.Button(sensor_frame, text="Get all AS5600", command=lambda: self.send("get as5600")).pack(
            side="left", padx=4
        )
        ttk.Label(sensor_frame, text="CH").pack(side="left", padx=(16, 2))
        ttk.Spinbox(sensor_frame, from_=0, to=7, textvariable=self.as5600_channel_var, width=4).pack(side="left")
        ttk.Button(sensor_frame, text="Get CH", command=self.get_as5600_channel).pack(side="left", padx=4)
        ttk.Label(sensor_frame, text="Stream CH").pack(side="left", padx=(16, 2))
        ttk.Spinbox(sensor_frame, from_=0, to=7, textvariable=self.stream_channel_var, width=4).pack(side="left")
        ttk.Label(sensor_frame, text="ms").pack(side="left", padx=(8, 2))
        ttk.Spinbox(sensor_frame, from_=50, to=5000, textvariable=self.stream_interval_var, width=6).pack(side="left")
        ttk.Button(sensor_frame, text="Start stream", command=self.start_stream).pack(side="left", padx=4)
        ttk.Button(sensor_frame, text="Stream off", command=lambda: self.send("stream off")).pack(side="left", padx=4)

        rows = ttk.Frame(tab)
        rows.pack(fill="both", expand=True, pady=(8, 0))
        for index, axis in enumerate(POSITION_AXES):
            frame = ttk.LabelFrame(rows, text=AXIS_NAMES[axis - 1])
            frame.pack(fill="x", pady=4)
            self._build_position_axis_row(frame, index, axis)

    def _build_position_axis_row(self, frame: ttk.LabelFrame, index: int, axis: int) -> None:
        ttk.Button(frame, text="Zero", command=lambda: self.send(f"zero {axis}")).grid(
            row=0, column=0, padx=4, pady=4
        )
        ttk.Label(frame, text="Angle").grid(row=0, column=1, sticky="e")
        ttk.Entry(frame, textvariable=self.position_angle_vars[index], width=8).grid(row=0, column=2, padx=4)
        ttk.Button(frame, text="Goto", command=lambda: self.goto_axis(axis, index)).grid(row=0, column=3, padx=4)
        ttk.Label(frame, text="Rotate").grid(row=0, column=4, sticky="e")
        ttk.Entry(frame, textvariable=self.rotate_angle_vars[index], width=8).grid(row=0, column=5, padx=4)
        ttk.Button(frame, text="Move", command=lambda: self.rotate_axis(axis, index)).grid(row=0, column=6, padx=4)
        ttk.Button(frame, text="Stop", command=lambda: self.send(f"stop {axis}")).grid(row=0, column=7, padx=4)
        ttk.Button(frame, text="Hold now", command=lambda: self.hold_axis(axis)).grid(row=0, column=8, padx=4)

        ttk.Label(frame, text="Stopmode").grid(row=1, column=0, sticky="e", padx=4)
        ttk.Combobox(frame, textvariable=self.stopmode_vars[index], values=STOP_MODES, width=8, state="readonly").grid(
            row=1, column=1, padx=4
        )
        ttk.Button(frame, text="Set", command=lambda: self.set_stopmode(axis, index)).grid(row=1, column=2, padx=4)
        ttk.Label(frame, text="KP").grid(row=1, column=3, sticky="e")
        ttk.Spinbox(frame, from_=0, to=10000, textvariable=self.kp_vars[index], width=7).grid(row=1, column=4, padx=4)
        ttk.Label(frame, text="KD").grid(row=1, column=5, sticky="e")
        ttk.Spinbox(frame, from_=0, to=10000, textvariable=self.kd_vars[index], width=7).grid(row=1, column=6, padx=4)
        ttk.Button(frame, text="Set KP/KD", command=lambda: self.set_kp_kd(axis, index)).grid(
            row=1, column=7, padx=4
        )

        ttk.Label(frame, text="MinPWM").grid(row=2, column=0, sticky="e", padx=4)
        ttk.Spinbox(frame, from_=0, to=255, textvariable=self.minpwm_vars[index], width=7).grid(
            row=2, column=1, padx=4
        )
        ttk.Label(frame, text="Tol deg").grid(row=2, column=2, sticky="e")
        ttk.Entry(frame, textvariable=self.tolerance_vars[index], width=7).grid(row=2, column=3, padx=4)
        ttk.Label(frame, text="AS5600 CH").grid(row=2, column=4, sticky="e")
        ttk.Spinbox(frame, from_=-1, to=7, textvariable=self.as5600_vars[index], width=5).grid(
            row=2, column=5, padx=4
        )
        ttk.Button(frame, text="Set PID/sensor", command=lambda: self.set_pid_sensor(axis, index)).grid(
            row=2, column=6, padx=4
        )
        ttk.Button(frame, text="Active hold preset", command=lambda: self.active_hold_preset(axis, index)).grid(
            row=2, column=7, padx=4
        )

    def _build_tuning_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text="Tuning")

        select = ttk.Frame(tab)
        select.pack(fill="x")
        ttk.Label(select, text="Joint").pack(side="left")
        ttk.Spinbox(select, from_=1, to=6, textvariable=self.tune_axis_var, width=5).pack(side="left", padx=4)
        ttk.Button(select, text="Params", command=lambda: self.send("params")).pack(side="left", padx=4)
        ttk.Button(select, text="Get motors", command=lambda: self.send("get motors")).pack(side="left", padx=4)
        ttk.Button(select, text="Get fault", command=lambda: self.send("get fault")).pack(side="left", padx=4)

        general = ttk.LabelFrame(tab, text="General axis settings")
        general.pack(fill="x", pady=8)
        self._add_tune_scale(general, "invert", self.tune_invert_var, 0, 1, 0, "invert")
        self._add_tune_scale(general, "maxpwm", self.tune_maxpwm_var, 0, 255, 1, "maxpwm")
        self._add_tune_scale(general, "default", self.tune_default_var, 0, 255, 2, "default")
        self._add_tune_scale(general, "posgain", self.tune_posgain_var, 100, 3000, 3, "posgain")
        self._add_tune_scale(general, "neggain", self.tune_neggain_var, 100, 3000, 4, "neggain")
        self._add_tune_scale(general, "dead", self.tune_dead_var, 0, 400, 5, "dead")

        keys = ttk.LabelFrame(tab, text="Keybinds and position-only mapping")
        keys.pack(fill="x", pady=8)
        ttk.Label(keys, text="KEYFWD").grid(row=0, column=0, padx=4, pady=4)
        ttk.Spinbox(keys, from_=-1, to=32767, textvariable=self.tune_keyfwd_var, width=8).grid(row=0, column=1)
        ttk.Button(keys, text="Set keyfwd", command=lambda: self.set_tune_value("keybind", self.tune_keyfwd_var)).grid(
            row=0, column=2, padx=4
        )
        ttk.Label(keys, text="KEYBACK").grid(row=0, column=3, padx=4)
        ttk.Spinbox(keys, from_=-1, to=32767, textvariable=self.tune_keyback_var, width=8).grid(row=0, column=4)
        ttk.Button(
            keys,
            text="Set keyback",
            command=lambda: self.set_tune_value("negkeybind", self.tune_keyback_var),
        ).grid(row=0, column=5, padx=4)
        ttk.Label(keys, text="AS5600 CH J1-J3").grid(row=1, column=0, padx=4, pady=4)
        self.tune_as5600_var = tk.IntVar(value=0)
        ttk.Spinbox(keys, from_=-1, to=7, textvariable=self.tune_as5600_var, width=8).grid(row=1, column=1)
        ttk.Button(keys, text="Set as5600", command=lambda: self.set_tune_value("as5600", self.tune_as5600_var)).grid(
            row=1, column=2, padx=4
        )

        safety = ttk.LabelFrame(tab, text="J1-J3 multi-breakpoint boundaries")
        safety.pack(fill="x", pady=8)
        for column, text in enumerate(("Slot", "Mode", "Angle", "Apply", "Disable")):
            ttk.Label(safety, text=text).grid(
                row=0, column=column, padx=4, pady=3, sticky="w"
            )
        for slot in range(4):
            row = slot + 1
            ttk.Label(safety, text=f"BP{slot + 1}").grid(
                row=row, column=0, padx=4, pady=3, sticky="w"
            )
            ttk.Combobox(
                safety,
                textvariable=self.tune_bp_slot_mode_vars[slot],
                values=BREAKPOINT_MODES,
                width=17,
                state="readonly",
            ).grid(row=row, column=1, padx=4)
            ttk.Entry(
                safety,
                textvariable=self.tune_bp_slot_angle_vars[slot],
                width=9,
            ).grid(row=row, column=2, padx=4)
            ttk.Button(
                safety,
                text="Apply",
                command=lambda selected_slot=slot: self.set_bp_slot(selected_slot),
            ).grid(row=row, column=3, padx=4)
            ttk.Button(
                safety,
                text="Off",
                command=lambda selected_slot=slot: self.disable_bp_slot(selected_slot),
            ).grid(row=row, column=4, padx=4)

        ttk.Label(
            safety,
            text="BP settings can only change while the selected joint is stopped.",
        ).grid(row=5, column=0, columnspan=5, padx=4, pady=(3, 5), sticky="w")

        bp_model = ttk.LabelFrame(tab, text="3-layer brakepoint model (J1-J3 only)")
        bp_model.pack(fill="x", pady=8)
        bp_settings = (
            ("Reaction (ms)", "bpreact", self.tune_bp_react_var, 0, 500),
            ("Decel (deg/s²)", "bpdecel", self.tune_bp_decel_var, 50, 20000),
            ("Lead margin (deg)", "bplead", self.tune_bp_lead_var, None, None),
            ("Recovery clearance (deg)", "bpclear", self.tune_bp_clear_var, None, None),
            ("Recovery max PWM", "bprecpwm", self.tune_bp_recpwm_var, 0, 255),
            ("Recovery timeout (ms)", "bprectime", self.tune_bp_rectime_var, 250, 10000),
        )
        for index, (label, command, variable, minimum, maximum) in enumerate(bp_settings):
            row = index // 2
            column = (index % 2) * 3
            ttk.Label(bp_model, text=label).grid(row=row, column=column, sticky="e", padx=4, pady=4)
            if minimum is None or maximum is None:
                ttk.Entry(bp_model, textvariable=variable, width=10).grid(row=row, column=column + 1, padx=4)
            else:
                ttk.Spinbox(
                    bp_model,
                    from_=minimum,
                    to=maximum,
                    textvariable=variable,
                    width=10,
                ).grid(row=row, column=column + 1, padx=4)
            ttk.Button(
                bp_model,
                text=f"Set {command}",
                command=lambda name=command, var=variable: self.set_bp_tune_setting(name, var),
            ).grid(row=row, column=column + 2, padx=4)
        ttk.Label(
            bp_model,
            text="Early-brake lead = speed × reaction + speed² / (2 × decel) + lead margin",
        ).grid(row=3, column=0, columnspan=6, sticky="w", padx=4, pady=(5, 2))
        ttk.Label(bp_model, textvariable=self.tune_bp_status_var).grid(
            row=4, column=0, columnspan=4, sticky="w", padx=4, pady=(2, 6)
        )
        ttk.Button(bp_model, text="Set all BP tuning", command=self.set_all_bp_tuning).grid(
            row=4, column=4, padx=4, pady=(2, 6)
        )
        ttk.Button(bp_model, text="Params / load", command=lambda: self.send("params")).grid(
            row=4, column=5, padx=4, pady=(2, 6)
        )

    def _build_drill_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text="DRILL")

        top = ttk.Frame(tab)
        top.pack(fill="x")
        ttk.Button(top, text="Mode DRILL", command=lambda: self.send("mode drill confirm")).pack(side="left", padx=4)
        ttk.Button(top, text="Mode SAFE", command=lambda: self.send("mode safe")).pack(side="left", padx=4)
        ttk.Button(top, text="StopAll", style="Danger.TButton", command=lambda: self.send("stopall")).pack(
            side="left", padx=4
        )

        elv = ttk.LabelFrame(tab, text="Elevator: BTS4 + BTS5")
        elv.pack(fill="x", pady=8)
        self._build_drill_pwm_row(
            elv,
            self.elv_pwm_var,
            [("Up", lambda: self.send(f"elv up {self.elv_pwm_var.get()}")),
             ("Down", lambda: self.send(f"elv down {self.elv_pwm_var.get()}")),
             ("Stop", lambda: self.send("elv stop"))],
        )
        self._build_drill_settings_row(
            elv,
            "elv",
            self.elv_default_var,
            self.elv_max_var,
            self.elv_invert_var,
            row=1,
        )

        servo = ttk.LabelFrame(tab, text="Tube selector servo: Power HD 1235MG")
        servo.pack(fill="x", pady=8)
        ttk.Label(servo, textvariable=self.servo_state_display_var).grid(
            row=0, column=0, columnspan=2, sticky="w", padx=4, pady=6
        )
        ttk.Label(servo, textvariable=self.servo_motion_display_var).grid(
            row=0, column=2, columnspan=2, sticky="w", padx=4, pady=6
        )
        ttk.Label(servo, textvariable=self.servo_angle_display_var).grid(
            row=0, column=4, columnspan=2, sticky="w", padx=4, pady=6
        )
        ttk.Label(servo, textvariable=self.servo_safety_display_var).grid(
            row=0, column=6, columnspan=2, sticky="w", padx=4, pady=6
        )
        for state in range(1, SERVO_STATE_COUNT + 1):
            ttk.Button(
                servo,
                text=servo_state_label(state),
                command=lambda target=state: self.send(f"servo {target}"),
            ).grid(row=1, column=state - 1, padx=4, pady=6)
        ttk.Button(
            servo, text="Servo Wiggle", command=lambda: self.send("servowiggle")
        ).grid(row=1, column=5, padx=4)

        ttk.Label(servo, textvariable=self.servo_reference_display_var).grid(
            row=2, column=0, columnspan=3, sticky="w", padx=4, pady=6
        )
        ttk.Button(
            servo,
            text="1000 us",
            command=lambda: self.send("servo pulse 1000"),
        ).grid(row=2, column=3, padx=4)
        ttk.Button(
            servo,
            text="1500 us",
            command=lambda: self.send("servo pulse 1500"),
        ).grid(row=2, column=4, padx=4)
        ttk.Button(
            servo,
            text="2000 us",
            command=lambda: self.send("servo pulse 2000"),
        ).grid(row=2, column=5, padx=4)
        ttk.Button(
            servo,
            text="PWM off",
            command=lambda: self.send("servo pulse off"),
        ).grid(row=2, column=6, padx=4)
        ttk.Button(
            servo,
            text="Timer diag",
            command=lambda: self.send("servo diag"),
        ).grid(row=2, column=7, padx=4)

        ttk.Label(
            servo, textvariable=self.servo_calibration_display_var
        ).grid(row=3, column=0, columnspan=4, sticky="w", padx=4, pady=6)
        for state in range(1, SERVO_STATE_COUNT + 1):
            ttk.Button(
                servo,
                text=f"Save current as {servo_state_label(state)}",
                command=lambda target=state: self.send(
                    f"set servostate {target}"
                ),
            ).grid(row=3, column=state + 3, padx=4, pady=4)
        ttk.Button(
            servo,
            text="Clear tube calibration",
            command=lambda: self.send("set servocal clear"),
        ).grid(row=4, column=0, columnspan=2, padx=4, pady=4)
        ttk.Button(
            servo,
            text="Save calibration to flash",
            command=lambda: self.send("save"),
        ).grid(row=4, column=2, columnspan=2, padx=4, pady=4)
        ttk.Label(servo, textvariable=self.servo_manual_display_var).grid(
            row=7, column=0, columnspan=9, sticky="w", padx=4
        )

        servo_settings = (
            ("Wiggle angle", self.servo_wiggle_angle_var, "servowiggleangle", 1, 30),
        )
        for row, (label, variable, command, low, high) in enumerate(
            servo_settings, start=5
        ):
            ttk.Label(servo, text=label).grid(row=row, column=0, padx=4, pady=4)
            ttk.Spinbox(
                servo, from_=low, to=high, textvariable=variable, width=6
            ).grid(row=row, column=1, padx=4)
            ttk.Button(
                servo,
                text="Set",
                command=lambda name=command, var=variable: self.send(
                    f"set {name} {var.get()}"
                ),
            ).grid(row=row, column=2, padx=4)
        ttk.Label(servo, text="Invert").grid(row=5, column=3, padx=4)
        ttk.Spinbox(
            servo, from_=0, to=1, textvariable=self.servo_invert_var, width=5
        ).grid(row=5, column=4, padx=4)
        ttk.Button(
            servo,
            text="Set invert",
            command=lambda: self.send(f"set servo invert {self.servo_invert_var.get()}"),
        ).grid(row=5, column=5, padx=4)
        ttk.Label(servo, text="Speed (deg/s)").grid(row=5, column=6, padx=4)
        ttk.Spinbox(
            servo, from_=5, to=120, textvariable=self.servo_speed_var, width=6
        ).grid(row=5, column=7, padx=4)
        ttk.Button(
            servo,
            text="Set speed",
            command=lambda: self.send(f"set servospeed {self.servo_speed_var.get()}"),
        ).grid(row=5, column=8, padx=4)
        ttk.Label(servo, text="Custom fixed pulse (us)").grid(
            row=6, column=0, padx=4, pady=4
        )
        ttk.Spinbox(
            servo,
            from_=800,
            to=2200,
            textvariable=self.servo_pulse_var,
            width=7,
        ).grid(row=6, column=1, padx=4)
        ttk.Button(
            servo,
            text="Apply once",
            command=lambda: self.send(
                f"servo pulse {self.servo_pulse_var.get()}"
            ),
        ).grid(row=6, column=2, padx=4)
        ttk.Label(servo, text="Manual step (us)").grid(
            row=6, column=3, padx=4, pady=4
        )
        ttk.Spinbox(
            servo,
            from_=1,
            to=100,
            textvariable=self.servo_step_var,
            width=6,
        ).grid(row=6, column=4, padx=4)
        ttk.Button(
            servo,
            text="Backward",
            command=lambda: self.send(
                f"servo backward {self.servo_step_var.get()}"
            ),
        ).grid(row=6, column=5, padx=4)
        ttk.Button(
            servo,
            text="Forward",
            command=lambda: self.send(
                f"servo forward {self.servo_step_var.get()}"
            ),
        ).grid(row=6, column=6, padx=4)

        leds = ttk.LabelFrame(tab, text="Drill LEDs / active-LOW relay module")
        leds.pack(fill="x", pady=8)
        for index in range(4):
            ttk.Label(leds, textvariable=self.drill_led_state_vars[index]).grid(
                row=index, column=0, sticky="w", padx=4, pady=4
            )
            ttk.Button(
                leds,
                text="ON",
                command=lambda led=index + 1: self.send(f"DrillLed {led} 1"),
            ).grid(row=index, column=1, padx=4)
            ttk.Button(
                leds,
                text="OFF",
                command=lambda led=index + 1: self.send(f"DrillLed {led} 0"),
            ).grid(row=index, column=2, padx=4)
        ttk.Label(elv, text="BTS4 sendpwm").grid(row=2, column=0, padx=4, pady=6)
        ttk.Spinbox(
            elv,
            from_=0.0,
            to=1.0,
            increment=0.01,
            format="%.2f",
            textvariable=self.elv_sendpwm_vars[0],
            width=6,
        ).grid(row=2, column=1)
        ttk.Button(
            elv,
            text="Set BTS4",
            command=lambda: self.set_elevator_sendpwm(4, 0),
        ).grid(row=2, column=2, padx=4)
        ttk.Label(elv, text="BTS5 sendpwm").grid(row=2, column=3, padx=4)
        ttk.Spinbox(
            elv,
            from_=0.0,
            to=1.0,
            increment=0.01,
            format="%.2f",
            textvariable=self.elv_sendpwm_vars[1],
            width=6,
        ).grid(row=2, column=4)
        ttk.Button(
            elv,
            text="Set BTS5",
            command=lambda: self.set_elevator_sendpwm(5, 1),
        ).grid(row=2, column=5, padx=4)
        ttk.Button(elv, text="Params / load", command=lambda: self.send("params")).grid(
            row=2, column=8, padx=4
        )

        drill = ttk.LabelFrame(tab, text="Drill: BTS6")
        drill.pack(fill="x", pady=8)
        self._build_drill_pwm_row(
            drill,
            self.drill_pwm_var,
            [("Dig", lambda: self.send(f"drill dig {self.drill_pwm_var.get()}")),
             ("Extract", lambda: self.send(f"drill extract {self.drill_pwm_var.get()}")),
             ("Stop", lambda: self.send("drill stop"))],
        )
        self._build_drill_settings_row(
            drill,
            "drill",
            self.drill_default_var,
            self.drill_max_var,
            self.drill_invert_var,
            row=1,
        )

    def _build_joy_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text="Joy/Keybind Test")

        axis_frame = ttk.LabelFrame(tab, text="joy axis")
        axis_frame.pack(fill="x")
        ttk.Label(axis_frame, text="Target (J1-J6)").pack(side="left", padx=4, pady=6)
        ttk.Spinbox(axis_frame, from_=1, to=6, textvariable=self.joy_axis_var, width=5).pack(side="left")
        ttk.Scale(axis_frame, from_=-1000, to=1000, variable=self.joy_value_var, orient="horizontal").pack(
            side="left", fill="x", expand=True, padx=8
        )
        ttk.Button(axis_frame, text="Send joy axis", command=self.send_joy_axis).pack(side="left", padx=4)
        ttk.Button(axis_frame, text="Zero + send", command=self.zero_joy_axis).pack(side="left", padx=4)

        button_frame = ttk.LabelFrame(tab, text="joy button")
        button_frame.pack(fill="x", pady=8)
        ttk.Label(button_frame, text="Button id").pack(side="left", padx=4, pady=6)
        ttk.Spinbox(button_frame, from_=0, to=32767, textvariable=self.joy_button_id_var, width=8).pack(side="left")
        ttk.Button(button_frame, text="Press", command=lambda: self.send_joy_button(1)).pack(side="left", padx=4)
        ttk.Button(button_frame, text="Release", command=lambda: self.send_joy_button(0)).pack(side="left", padx=4)

        preset_frame = ttk.LabelFrame(tab, text="D-pad synthetic ids")
        preset_frame.pack(fill="x", pady=8)
        for label, button_id in (("Hat up 1000", 1000), ("Hat down 1001", 1001), ("Hat left 1002", 1002), ("Hat right 1003", 1003)):
            ttk.Button(preset_frame, text=f"{label} press", command=lambda bid=button_id: self.send(f"joy button {bid} 1")).pack(
                side="left", padx=4, pady=6
            )
            ttk.Button(preset_frame, text="release", command=lambda bid=button_id: self.send(f"joy button {bid} 0")).pack(
                side="left", padx=(0, 8), pady=6
            )

    def _build_gamepad_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text="Gamepad ARM")

        controls = ttk.LabelFrame(tab, text="Gamepad connection", padding=6)
        controls.pack(fill="x")
        ttk.Button(controls, text="Detect gamepads", command=self.detect_gamepads).pack(side="left", padx=4)
        self.gamepad_combo = ttk.Combobox(
            controls,
            textvariable=self.gamepad_device_var,
            state="readonly",
            width=62,
        )
        self.gamepad_combo.pack(side="left", fill="x", expand=True, padx=4)
        ttk.Button(controls, text="Start gamepad TX", command=self.start_gamepad_mode).pack(side="left", padx=4)
        ttk.Button(controls, text="Pause gamepad TX", command=self.stop_gamepad_mode).pack(side="left", padx=4)
        ttk.Button(controls, text="Mode ARM", command=lambda: self.send("mode arm confirm")).pack(side="left", padx=4)
        ttk.Button(controls, text="Mode DRILL", command=lambda: self.send("mode drill confirm")).pack(side="left", padx=4)
        ttk.Button(controls, text="Stop ARM", command=lambda: self.send("stop")).pack(side="left", padx=4)
        ttk.Button(controls, text="StopAll", style="Danger.TButton", command=lambda: self.send("stopall")).pack(
            side="left", padx=4
        )

        settings = ttk.Frame(tab, padding=(0, 8))
        settings.pack(fill="x")
        ttk.Label(settings, textvariable=self.gamepad_status_var).pack(side="left", padx=4)
        ttk.Label(settings, text="Axis dead").pack(side="left", padx=(24, 4))
        ttk.Spinbox(settings, from_=0, to=400, textvariable=self.gamepad_dead_var, width=6).pack(side="left")
        ttk.Label(settings, text="Axis delta").pack(side="left", padx=(12, 4))
        ttk.Spinbox(settings, from_=1, to=300, textvariable=self.gamepad_delta_var, width=6).pack(side="left")
        ttk.Label(settings, text="Poll ms").pack(side="left", padx=(12, 4))
        ttk.Spinbox(settings, from_=10, to=200, textvariable=self.gamepad_poll_var, width=6).pack(side="left")

        mappings = ttk.Frame(tab)
        mappings.pack(fill="both", expand=True)
        axis_frame = ttk.LabelFrame(mappings, text="Axis mapping", padding=6)
        axis_frame.pack(side="left", fill="both", expand=True, padx=(0, 4))
        for row, (name, _joint, _axis, _invert) in enumerate(GAMEPAD_AXIS_DEFAULTS):
            ttk.Label(axis_frame, text=name, width=10).grid(row=row, column=0, sticky="w", padx=4, pady=4)
            ttk.Label(axis_frame, text="Target").grid(row=row, column=1, padx=(8, 2))
            ttk.Spinbox(
                axis_frame,
                from_=1,
                to=6,
                textvariable=self.gamepad_axis_joint_vars[row],
                width=5,
            ).grid(row=row, column=2)
            ttk.Label(axis_frame, text="Gamepad axis").grid(row=row, column=3, padx=(8, 2))
            ttk.Spinbox(
                axis_frame,
                from_=0,
                to=15,
                textvariable=self.gamepad_axis_id_vars[row],
                width=5,
            ).grid(row=row, column=4)
            ttk.Checkbutton(
                axis_frame,
                text="Invert",
                variable=self.gamepad_axis_invert_vars[row],
            ).grid(row=row, column=5, padx=8)

        button_frame = ttk.LabelFrame(mappings, text="Button / D-pad mapping", padding=6)
        button_frame.pack(side="left", fill="both", expand=True, padx=(4, 0))
        for row, (name, _button, _joy_id) in enumerate(GAMEPAD_BUTTON_DEFAULTS):
            ttk.Label(button_frame, text=name, width=12).grid(row=row, column=0, sticky="w", padx=4, pady=4)
            ttk.Label(button_frame, text="Button").grid(row=row, column=1, padx=(8, 2))
            ttk.Spinbox(
                button_frame,
                from_=0,
                to=63,
                textvariable=self.gamepad_button_id_vars[row],
                width=5,
            ).grid(row=row, column=2)
            ttk.Label(button_frame, text="F401 joy id").grid(row=row, column=3, padx=(8, 2))
            ttk.Spinbox(
                button_frame,
                from_=0,
                to=32767,
                textvariable=self.gamepad_joy_id_vars[row],
                width=7,
            ).grid(row=row, column=4)
        ttk.Label(
            button_frame,
            text="D-pad hats are also mapped automatically to these four F401 joy ids.",
        ).grid(row=len(GAMEPAD_BUTTON_DEFAULTS), column=0, columnspan=5, sticky="w", padx=4, pady=(8, 2))

        live = ttk.LabelFrame(tab, text="Live gamepad values", padding=6)
        live.pack(fill="x", pady=(8, 0))
        ttk.Label(live, textvariable=self.gamepad_debug_var, justify="left").pack(anchor="w")

    def _build_drill_pwm_row(self, frame: ttk.LabelFrame, pwm_var: tk.IntVar, buttons: Iterable[tuple[str, Callable[[], None]]]) -> None:
        ttk.Label(frame, text="PWM").grid(row=0, column=0, padx=4, pady=6)
        ttk.Scale(frame, from_=0, to=255, variable=pwm_var, orient="horizontal").grid(
            row=0, column=1, sticky="ew", padx=4
        )
        ttk.Spinbox(frame, from_=0, to=255, textvariable=pwm_var, width=6).grid(row=0, column=2, padx=4)
        for col, (text, command) in enumerate(buttons, start=3):
            ttk.Button(frame, text=text, command=command).grid(row=0, column=col, padx=4)
        frame.columnconfigure(1, weight=1)

    def _build_drill_settings_row(
        self,
        frame: ttk.LabelFrame,
        prefix: str,
        default_var: tk.IntVar,
        max_var: tk.IntVar,
        invert_var: tk.IntVar,
        row: int,
    ) -> None:
        ttk.Label(frame, text="default").grid(row=row, column=0, padx=4, pady=6)
        ttk.Spinbox(frame, from_=0, to=255, textvariable=default_var, width=6).grid(row=row, column=1)
        ttk.Button(frame, text="Set default", command=lambda: self.send(f"{prefix} default {default_var.get()}")).grid(
            row=row, column=2, padx=4
        )
        ttk.Label(frame, text="maxpwm").grid(row=row, column=3, padx=4)
        ttk.Spinbox(frame, from_=0, to=255, textvariable=max_var, width=6).grid(row=row, column=4)
        ttk.Button(frame, text="Set maxpwm", command=lambda: self.send(f"{prefix} maxpwm {max_var.get()}")).grid(
            row=row, column=5, padx=4
        )
        ttk.Label(frame, text="invert").grid(row=row, column=6, padx=4)
        ttk.Spinbox(frame, from_=0, to=1, textvariable=invert_var, width=4).grid(row=row, column=7)
        ttk.Button(frame, text="Set invert", command=lambda: self.send(f"{prefix} invert {invert_var.get()}")).grid(
            row=row, column=8, padx=4
        )

    def _button_row(self, parent: ttk.Frame, buttons: Iterable[tuple[str, str]]) -> None:
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=2)
        for text, command in buttons:
            style = "Danger.TButton" if "Stop" in text or "STOP" in text else "TButton"
            ttk.Button(row, text=text, style=style, command=lambda cmd=command: self.send(cmd)).pack(
                side="left", padx=3
            )

    def _add_tune_scale(
        self,
        parent: ttk.LabelFrame,
        label: str,
        variable: tk.IntVar,
        low: int,
        high: int,
        row: int,
        command_name: str,
    ) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="e", padx=4, pady=4)
        ttk.Scale(parent, from_=low, to=high, variable=variable, orient="horizontal").grid(
            row=row, column=1, sticky="ew", padx=4
        )
        ttk.Spinbox(parent, from_=low, to=high, textvariable=variable, width=8).grid(row=row, column=2, padx=4)
        ttk.Button(parent, text="Set", command=lambda: self.set_tune_value(command_name, variable)).grid(
            row=row, column=3, padx=4
        )
        parent.columnconfigure(1, weight=1)

    def refresh_ports(self) -> None:
        ports = serial_ports()
        self.port_combo["values"] = ports
        if not self.port_var.get() and ports:
            self.port_var.set(ports[0])

    def connect(self) -> None:
        try:
            self.link.connect(self.port_var.get(), int(self.baud_var.get()), self.dry_run_var.get())
        except Exception as exc:
            messagebox.showerror("Connect failed", str(exc))
            self.log(f"LOCAL: connect failed: {exc}")
            return
        self.connected_var.set("Connected" if not self.dry_run_var.get() else "Dry run")

    def disconnect(self) -> None:
        if self.gamepad_enabled:
            self.stop_gamepad_mode()
        elif self.link.is_open():
            self.send("failsafe linkloss")
        self.link.close()
        self.connected_var.set("Disconnected")
        self.log("LOCAL: disconnected")

    def log(self, text: str) -> None:
        if not hasattr(self, "log_text"):
            print(text, flush=True)
            return
        self.log_text.configure(state="normal")
        self.log_text.insert("end", text + "\n")
        self._log_line_count += text.count("\n") + 1
        if self._log_line_count > MAX_DESKTOP_LOG_LINES:
            lines_to_remove = self._log_line_count - DESKTOP_LOG_LINES_AFTER_TRIM
            self.log_text.delete("1.0", f"{lines_to_remove + 1}.0")
            self._log_line_count -= lines_to_remove
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def clear_log(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")
        self._log_line_count = 0

    def send(self, command: str) -> None:
        self.link.send(command)

    def send_many(self, commands: Iterable[str]) -> None:
        for command in commands:
            self.send(command)

    def send_manual(self) -> None:
        command = self.command_var.get().strip()
        if command:
            self.send(command)
            self.command_var.set("")

    def send_seq(self) -> None:
        self.send(f"seq {self.seq_id_var.get()} {self.seq_command_var.get().strip()}")
        self.seq_id_var.set(self.seq_id_var.get() + 1)

    def poll_serial(self) -> None:
        for line in self.link.read_lines():
            if not self.update_status_from_line(line):
                self.log(line if line.startswith("LOCAL:") else f"F401: {line}")
        if not self.link.is_open() and self.connected_var.get() not in ("Disconnected", "Dry run"):
            self.connected_var.set("Disconnected")
        self.root.after(30, self.poll_serial)

    def service_heartbeat(self) -> None:
        if self.link.is_open():
            self.link.send("hb", log_tx=False)
        self.root.after(
            int(HEARTBEAT_INTERVAL_SECONDS * 1000), self.service_heartbeat
        )

    def update_status_from_line(self, line: str) -> bool:
        status = parse_status_line(line)
        if status is not None:
            self.apply_live_status(status)
            return True
        bp_tuning = parse_brakepoint_tuning_line(line)
        if bp_tuning is not None:
            axis, values = bp_tuning
            self.apply_brakepoint_tuning(axis, values)
        bp_slot = parse_brakepoint_slot_line(line)
        if bp_slot is not None:
            axis, slot, values = bp_slot
            self.apply_brakepoint_slot(axis, slot, values)
        elevator_params = parse_elevator_params_line(line)
        if elevator_params is not None:
            self.elv_sendpwm_vars[0].set(elevator_params["SENDPWM_BTS4"])
            self.elv_sendpwm_vars[1].set(elevator_params["SENDPWM_BTS5"])
        servo_status = parse_servo_status_line(line)
        if servo_status is not None:
            self.apply_servo_status(servo_status)
            return True
        servo_extended = parse_servo_extended_line(line)
        if servo_extended is not None:
            self.apply_servo_extended_status(servo_extended)
            return True
        drill_leds = parse_drill_led_status_line(line)
        if drill_leds is not None:
            self.apply_drill_led_status(drill_leds)
            return True
        if line.startswith("OK MODE "):
            self.last_mode_var.set("Mode: " + line.removeprefix("OK MODE ").strip())
        elif line.startswith("OK FAULT "):
            parts = line.split()
            if len(parts) >= 3:
                self.last_fault_var.set("Fault: " + " ".join(parts[2:5]))
        return False

    def apply_live_status(self, status: Dict[str, object]) -> None:
        mode = str(status.get("mode", "-"))
        stopmode = str(status.get("stopmode", "-"))
        self.live_status_mode_var.set("Mode: " + mode)
        self.live_status_stopmode_var.set("Stopmode: " + stopmode)
        self.live_status_seen_var.set("Live status: " + time.strftime("%H:%M:%S"))
        self.last_mode_var.set("Mode: " + mode)
        axes = status.get("axes", [])
        if not isinstance(axes, list):
            return
        for index, axis in enumerate(axes[: len(self.live_status_vars)]):
            if not isinstance(axis, dict):
                continue
            for field in STATUS_FIELDS:
                self.live_status_vars[index][field].set(str(axis.get(field, "-")))
        servo = status.get("servo", {})
        if isinstance(servo, dict) and servo.get("SERVO_INIT", "-") != "-":
            self.apply_servo_status(
                {field: str(servo.get(field, "-")) for field in SERVO_STATUS_FIELDS}
            )
            self.apply_servo_extended_status(
                {
                    field: str(servo.get(field, "-"))
                    for field in SERVO_EXTENDED_FIELDS
                    if servo.get(field, "-") != "-"
                }
            )
        led_states = status.get("drill_leds", [])
        if isinstance(led_states, list) and led_states and led_states[0] != "-":
            self.apply_drill_led_status([str(value) for value in led_states[:4]])

    def apply_servo_status(self, servo: Dict[str, str]) -> None:
        initialized = str(servo.get("SERVO_INIT", "0")) == "1"
        state = str(servo.get("SERVO_STATE", "NA"))
        target = str(servo.get("SERVO_TARGET", "NA"))
        moving = str(servo.get("SERVO_MOVING", "0")) == "1"
        self.servo_state_display_var.set(
            (
                f"Servo state: {servo_state_label(state)} → "
                f"target {servo_state_label(target)}"
            )
            if initialized
            else "Servo state: not initialized"
        )
        self.servo_motion_display_var.set(
            "Servo: moving" if moving else "Servo: idle"
        )
        self.servo_safety_display_var.set(
            f"Safety: {servo.get('SERVO_SAFETY', '-')}"
        )
        try:
            self.servo_angle_var.set(int(str(servo.get("SERVOANGLE", "45"))))
            self.servo_wiggle_angle_var.set(
                int(str(servo.get("SERVOWIGGLEANGLE", "5")))
            )
            self.servo_invert_var.set(
                int(str(servo.get("SERVO_INVERT", "0")))
            )
            self.servo_speed_var.set(
                int(str(servo.get("SERVOSPEED", "30")))
            )
        except ValueError:
            pass

    def apply_servo_extended_status(self, servo: Dict[str, str]) -> None:
        if "SERVO_REFERENCE_VALID" in servo:
            valid = servo.get("SERVO_REFERENCE_VALID", "0") == "1"
            error = servo.get("SERVO_ERROR", "NONE")
            current_pulse = servo.get("SERVO_CURRENT_PULSE", "NA")
            if error == "TARGET_PULSE_RANGE":
                reference_text = (
                    f"Calibration rejected at {current_pulse} us: "
                    "tube pulses must remain in state order"
                )
            elif error == "TARGET_NOT_CALIBRATED":
                reference_text = (
                    "Target tube is not calibrated; apply a fixed pulse "
                    "and save that tube first"
                )
            elif valid:
                reference_text = (
                    f"Current state reference: ready at "
                    f"{servo.get('SERVO_REFERENCE_PULSE', current_pulse)} us"
                )
            else:
                reference_text = (
                    f"Current state: not declared | pulse: {current_pulse} us"
                )
            self.servo_reference_display_var.set(reference_text)
        if "SERVO_MANUAL_VALUE" in servo:
            self.servo_manual_display_var.set(
                "State/fixed moves: speed-controlled ramp | step buttons: one click"
            )
            try:
                self.servo_speed_var.set(
                    int(servo.get("SERVO_CONFIGURED_SPEED", "30"))
                )
            except ValueError:
                pass
        if "SERVO_CURRENT_ANGLE" in servo:
            current = servo.get("SERVO_CURRENT_ANGLE", "NA")
            target = servo.get("SERVO_TARGET_ANGLE", "NA")
            reference = servo.get("SERVO_REFERENCE_ANGLE", "NA")
            if current == "NA":
                text = "Estimated PWM angle: unavailable"
            else:
                text = (
                    f"Estimated PWM angle: {current}° | target: {target}°"
                    f" | calibration reference: {reference}°"
                )
            self.servo_angle_display_var.set(text)
        if "SERVO_CAL_MASK" in servo:
            pulses = [
                servo.get(f"SERVO_CAL_P{state}", "NA")
                for state in range(1, SERVO_STATE_COUNT + 1)
            ]
            saved = sum(value != "NA" for value in pulses)
            self.servo_calibration_display_var.set(
                f"Servo calibration: {saved}/{SERVO_STATE_COUNT} saved | "
                + " | ".join(
                    (
                        f"{servo_state_label(state)}="
                        f"{pulses[state - 1]} us"
                    )
                    for state in range(1, SERVO_STATE_COUNT + 1)
                )
            )

    def apply_drill_led_status(self, led_states: List[str]) -> None:
        for index, value in enumerate(led_states[:4]):
            state_text = "ON" if str(value) == "1" else "OFF"
            self.drill_led_state_vars[index].set(
                f"Drill LED {index + 1}: {state_text}"
            )

    def safe_stop_save(self) -> None:
        self.send_many(("stopall", "mode safe", "save"))

    def set_elevator_sendpwm(self, bts: int, index: int) -> None:
        raw = self.elv_sendpwm_vars[index].get().strip()
        try:
            value = float(raw)
        except ValueError:
            messagebox.showerror("Invalid sendpwm", "sendpwm must be between 0.00 and 1.00.")
            return
        scaled = value * 100.0
        rounded = round(scaled)
        if (
            not math.isfinite(value)
            or value < 0.0
            or value > 1.0
            or abs(scaled - rounded) > 1e-6
        ):
            messagebox.showerror(
                "Invalid sendpwm",
                "sendpwm must be between 0.00 and 1.00 with 0.01 precision.",
            )
            return
        normalized = f"{rounded / 100.0:.2f}"
        self.elv_sendpwm_vars[index].set(normalized)
        self.send(f"set bts{bts} sendpwm {normalized}")

    def confirm_factory_reset(self) -> None:
        if messagebox.askyesno("Factory reset", "Reset RAM settings to firmware defaults?"):
            self.send("factoryreset confirm")

    def drive_axis(self, axis: int, direction: str) -> None:
        pwm = clamp_int(int(self.axis_pwm_vars[axis - 1].get()), 0, 255)
        self.send(f"{direction} {axis} {pwm}")

    def on_axis_slider(self, axis: int, value: str) -> None:
        index = axis - 1
        state = self.axis_slider_state[index]
        state.value = int(float(value))
        if not self.live_slider_var.get():
            return
        now = time.monotonic()
        if now - state.sent_at < 0.06 and abs(state.value - state.sent_value) < 40:
            return
        self.send_axis_slider(axis)

    def send_axis_slider(self, axis: int) -> None:
        index = axis - 1
        value = clamp_int(int(self.axis_slider_vars[index].get()), -1000, 1000)
        self.axis_slider_state[index].sent_value = value
        self.axis_slider_state[index].sent_at = time.monotonic()
        self.send(f"axis {axis} {value}")

    def zero_axis_slider(self, axis: int) -> None:
        self.axis_slider_vars[axis - 1].set(0)
        self.send(f"axis {axis} 0")
        self.send(f"stop {axis}")

    def goto_axis(self, axis: int, index: int) -> None:
        self.send(f"goto {axis} {self.position_angle_vars[index].get().strip()}")

    def rotate_axis(self, axis: int, index: int) -> None:
        self.send(f"rotate {axis} {self.rotate_angle_vars[index].get().strip()}")

    def hold_axis(self, axis: int) -> None:
        self.send_many((f"set {axis} stopmode hold", f"stop {axis}"))

    def set_stopmode(self, axis: int, index: int) -> None:
        self.send(f"set {axis} stopmode {self.stopmode_vars[index].get()}")

    def set_kp_kd(self, axis: int, index: int) -> None:
        self.send_many((f"set {axis} kp {self.kp_vars[index].get()}", f"set {axis} kd {self.kd_vars[index].get()}"))

    def set_pid_sensor(self, axis: int, index: int) -> None:
        self.send_many(
            (
                f"set {axis} minpwm {self.minpwm_vars[index].get()}",
                f"set {axis} tolerance {self.tolerance_vars[index].get().strip()}",
                f"set {axis} as5600 {self.as5600_vars[index].get()}",
            )
        )

    def active_hold_preset(self, axis: int, index: int) -> None:
        self.send_many(
            (
                "mode arm confirm",
                f"set {axis} as5600 {self.as5600_vars[index].get()}",
                f"set {axis} stopmode hold",
                f"set {axis} tolerance {self.tolerance_vars[index].get().strip()}",
                f"set {axis} kp {self.kp_vars[index].get()}",
                f"set {axis} kd {self.kd_vars[index].get()}",
                f"set {axis} minpwm {self.minpwm_vars[index].get()}",
                f"stop {axis}",
            )
        )

    def get_as5600_channel(self) -> None:
        self.send(f"get as5600 {self.as5600_channel_var.get()}")

    def start_stream(self) -> None:
        self.send(f"stream as5600 {self.stream_channel_var.get()} {self.stream_interval_var.get()}")

    def selected_tune_axis(self) -> int:
        return clamp_int(int(self.tune_axis_var.get()), 1, 6)

    def selected_brakepoint_axis(self) -> Optional[int]:
        axis = self.selected_tune_axis()
        if axis <= 3:
            return axis
        message = f"J{axis}: the 3-layer brakepoint model is available only on J1-J3."
        self.tune_bp_status_var.set(message)
        self.log("LOCAL: " + message)
        return None

    def on_tune_axis_changed(self, *_args: object) -> None:
        try:
            axis = self.selected_tune_axis()
        except (tk.TclError, TypeError, ValueError):
            return
        if axis > 3:
            self.tune_bp_status_var.set(f"J{axis}: BP model settings are unavailable (J1-J3 only).")
            return
        values = self.brakepoint_tuning_by_axis[axis]
        self.tune_bp_react_var.set(int(values["BPREACT_MS"]))
        self.tune_bp_decel_var.set(int(values["BPDECEL_DPS2"]))
        self.tune_bp_lead_var.set(values["BPLEAD_DEG"])
        self.tune_bp_clear_var.set(values["BPCLEAR_DEG"])
        self.tune_bp_recpwm_var.set(int(values["BPRECPWM"]))
        self.tune_bp_rectime_var.set(int(values["BPRECTIME_MS"]))
        for slot, slot_values in enumerate(self.brakepoint_slots_by_axis[axis]):
            self.tune_bp_slot_mode_vars[slot].set(slot_values["MODE"])
            angle = slot_values["ANGLE"]
            self.tune_bp_slot_angle_vars[slot].set("0" if angle == "NA" else angle)
        source = "last Params/local values" if axis in self.brakepoint_tuning_live_axes else "firmware defaults"
        self.tune_bp_status_var.set(f"J{axis}: {source} shown.")

    def apply_brakepoint_tuning(self, axis: int, values: Dict[str, str]) -> None:
        self.brakepoint_tuning_by_axis[axis] = values.copy()
        self.brakepoint_tuning_live_axes.add(axis)
        try:
            selected_axis = self.selected_tune_axis()
        except (tk.TclError, TypeError, ValueError):
            selected_axis = None
        if selected_axis == axis:
            self.on_tune_axis_changed()

    def apply_brakepoint_slot(self, axis: int, slot: int, values: Dict[str, str]) -> None:
        self.brakepoint_slots_by_axis[axis][slot - 1] = values.copy()
        if self.selected_tune_axis() == axis:
            self.on_tune_axis_changed()

    def set_bp_slot(self, slot: int) -> None:
        axis = self.selected_brakepoint_axis()
        if axis is None:
            return
        mode = self.tune_bp_slot_mode_vars[slot].get().strip().upper()
        if mode == "OFF":
            self.disable_bp_slot(slot)
            return
        angle = self.tune_bp_slot_angle_vars[slot].get().strip()
        command_mode = mode.lower()
        self.send(f"set {axis} bp {slot + 1} {command_mode} {angle}")
        self.brakepoint_slots_by_axis[axis][slot] = {
            "MODE": mode,
            "ANGLE": angle,
        }
        self.tune_bp_status_var.set(
            f"J{axis} BP{slot + 1}: configuration sent; Params verifies acceptance."
        )

    def disable_bp_slot(self, slot: int) -> None:
        axis = self.selected_brakepoint_axis()
        if axis is None:
            return
        self.send(f"set {axis} bp {slot + 1} off")
        self.tune_bp_slot_mode_vars[slot].set("OFF")
        self.brakepoint_slots_by_axis[axis][slot] = {
            "MODE": "OFF",
            "ANGLE": "NA",
        }
        self.tune_bp_status_var.set(
            f"J{axis} BP{slot + 1}: disable sent; Params verifies acceptance."
        )

    def set_bp_tune_setting(self, command_name: str, variable: tk.Variable) -> None:
        axis = self.selected_brakepoint_axis()
        if axis is None:
            return
        value = str(variable.get()).strip()
        self.send(f"set {axis} {command_name} {value}")
        field = BRAKEPOINT_COMMAND_TO_FIELD[command_name]
        self.brakepoint_tuning_by_axis[axis][field] = value
        self.brakepoint_tuning_live_axes.add(axis)
        self.tune_bp_status_var.set(f"J{axis}: {command_name} sent; use Params to verify.")

    def set_all_bp_tuning(self) -> None:
        axis = self.selected_brakepoint_axis()
        if axis is None:
            return
        commands = (
            f"set {axis} bpreact {self.tune_bp_react_var.get()}",
            f"set {axis} bpdecel {self.tune_bp_decel_var.get()}",
            f"set {axis} bplead {self.tune_bp_lead_var.get().strip()}",
            f"set {axis} bpclear {self.tune_bp_clear_var.get().strip()}",
            f"set {axis} bprecpwm {self.tune_bp_recpwm_var.get()}",
            f"set {axis} bprectime {self.tune_bp_rectime_var.get()}",
        )
        self.send_many(commands)
        self.brakepoint_tuning_by_axis[axis] = {
            "BPREACT_MS": str(self.tune_bp_react_var.get()),
            "BPDECEL_DPS2": str(self.tune_bp_decel_var.get()),
            "BPLEAD_DEG": self.tune_bp_lead_var.get().strip(),
            "BPCLEAR_DEG": self.tune_bp_clear_var.get().strip(),
            "BPRECPWM": str(self.tune_bp_recpwm_var.get()),
            "BPRECTIME_MS": str(self.tune_bp_rectime_var.get()),
        }
        self.brakepoint_tuning_live_axes.add(axis)
        self.tune_bp_status_var.set(f"J{axis}: all BP tuning commands sent; use Params to verify.")

    def send_tune_command(self, tail: str) -> None:
        self.send(f"set {self.selected_tune_axis()} {tail}")

    def set_tune_value(self, command_name: str, variable: tk.IntVar) -> None:
        self.send_tune_command(f"{command_name} {int(variable.get())}")

    def set_tune_text(self, command_name: str, variable: tk.StringVar) -> None:
        self.send_tune_command(f"{command_name} {variable.get().strip()}")

    def send_joy_axis(self) -> None:
        self.send(f"joy axis {self.joy_axis_var.get()} {self.joy_value_var.get()}")

    def zero_joy_axis(self) -> None:
        self.joy_value_var.set(0)
        self.send_joy_axis()

    def send_joy_button(self, state: int) -> None:
        self.send(f"joy button {self.joy_button_id_var.get()} {state}")

    @staticmethod
    def _gamepad_int(variable: tk.Variable, default: int) -> int:
        try:
            return int(variable.get())
        except (tk.TclError, TypeError, ValueError):
            return default

    def detect_gamepads(self, *, auto_start: bool = False) -> None:
        if self.gamepad_enabled:
            self.gamepad_status_var.set("Gamepad TX is running; pause it before detecting again.")
            return
        if self.gamepad_detecting:
            self.gamepad_start_after_detect = self.gamepad_start_after_detect or auto_start
            return
        self.gamepad_detecting = True
        self.gamepad_start_after_detect = auto_start
        self.gamepad_status_var.set("Detecting gamepads...")
        threading.Thread(target=self._detect_gamepads_worker, daemon=True).start()

    def _detect_gamepads_worker(self) -> None:
        try:
            executable = self.gamepad_python or find_pygame_python()
            if executable is None:
                raise RuntimeError(
                    "pygame was not found in the VS Code or PlatformIO Python environment."
                )
            command = [
                executable,
                str(Path(__file__).resolve()),
                "--gamepad-helper",
                "list",
            ]
            result = subprocess.run(
                command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=10,
                env=gamepad_subprocess_env(),
                creationflags=gamepad_subprocess_flags(),
                check=False,
            )
            payload: Optional[Dict[str, object]] = None
            for raw_line in reversed(result.stdout.splitlines()):
                try:
                    candidate = json.loads(raw_line)
                except json.JSONDecodeError:
                    continue
                if isinstance(candidate, dict):
                    payload = candidate
                    break
            if result.returncode != 0 or payload is None:
                detail = result.stdout.strip() or f"helper exited with code {result.returncode}"
                raise RuntimeError(detail)
            if payload.get("type") == "error":
                raise RuntimeError(str(payload.get("message", "gamepad helper failed")))
            devices = payload.get("devices")
            if not isinstance(devices, list):
                raise RuntimeError("gamepad helper returned an invalid device list")
            self.gamepad_messages.put(("devices", 0, (executable, devices)))
        except Exception as exc:
            self.gamepad_messages.put(("detect_error", 0, str(exc)))

    def _apply_detected_gamepads(self, payload: object) -> None:
        self.gamepad_detecting = False
        executable, raw_devices = payload  # type: ignore[misc]
        self.gamepad_python = str(executable)
        devices: Dict[int, Dict[str, object]] = {}
        labels: List[str] = []
        for raw_device in raw_devices:
            if not isinstance(raw_device, dict):
                continue
            try:
                index = int(raw_device["index"])
            except (KeyError, TypeError, ValueError):
                continue
            devices[index] = raw_device
            labels.append(
                f"{index}: {raw_device.get('name', 'Unknown gamepad')} "
                f"({raw_device.get('axes', 0)} axes, {raw_device.get('buttons', 0)} buttons, "
                f"{raw_device.get('hats', 0)} hats)"
            )
        self.gamepad_devices = devices
        self.gamepad_combo["values"] = labels
        if labels:
            selected_index = self._selected_gamepad_index()
            selected_label = next(
                (label for label in labels if label.startswith(f"{selected_index}:")),
                labels[0],
            )
            self.gamepad_device_var.set(selected_label)
            self.gamepad_status_var.set(f"Gamepads detected: {len(labels)}")
            if self.gamepad_start_after_detect:
                self.gamepad_start_after_detect = False
                self._start_selected_gamepad()
        else:
            self.gamepad_device_var.set("")
            self.gamepad_status_var.set("No gamepad detected. Move a stick or press a button, then detect again.")
            self.gamepad_start_after_detect = False

    def _selected_gamepad_index(self) -> int:
        prefix, separator, _tail = self.gamepad_device_var.get().partition(":")
        if separator:
            try:
                return int(prefix.strip())
            except ValueError:
                pass
        return min(self.gamepad_devices) if self.gamepad_devices else 0

    def start_gamepad_mode(self) -> None:
        if not self.gamepad_devices or self.gamepad_python is None:
            self.detect_gamepads(auto_start=True)
            return
        self._start_selected_gamepad()

    def _start_selected_gamepad(self) -> None:
        if self.gamepad_python is None:
            self.gamepad_status_var.set("Gamepad helper is not available; detect gamepads again.")
            return
        index = self._selected_gamepad_index()
        if index not in self.gamepad_devices:
            self.gamepad_status_var.set("Select a detected gamepad first.")
            return

        if self.gamepad_enabled:
            self.send("stop")
        self.gamepad_enabled = False
        self._stop_gamepad_process()
        self.gamepad_generation += 1
        generation = self.gamepad_generation
        poll_ms = clamp_int(self._gamepad_int(self.gamepad_poll_var, 20), 10, 200)
        command = [
            self.gamepad_python,
            str(Path(__file__).resolve()),
            "--gamepad-helper",
            "stream",
            "--gamepad-index",
            str(index),
            "--gamepad-helper-poll-ms",
            str(poll_ms),
        ]
        try:
            process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1,
                env=gamepad_subprocess_env(),
                creationflags=gamepad_subprocess_flags(),
            )
        except OSError as exc:
            self.gamepad_status_var.set(f"Could not start gamepad input: {exc}")
            return

        self.gamepad_process = process
        self.gamepad_enabled = True
        self.gamepad_latest_axes = []
        self.gamepad_last_axis.clear()
        self.gamepad_last_axis_sent_at.clear()
        self.gamepad_last_buttons.clear()
        device_name = str(self.gamepad_devices[index].get("name", f"Gamepad {index}"))
        self.gamepad_status_var.set(f"Gamepad TX: starting - {device_name}")
        threading.Thread(
            target=self._read_gamepad_stream,
            args=(process, generation),
            daemon=True,
        ).start()

    def _read_gamepad_stream(self, process: subprocess.Popen[str], generation: int) -> None:
        last_error = ""
        assert process.stdout is not None
        for raw_line in process.stdout:
            try:
                payload = json.loads(raw_line)
            except json.JSONDecodeError:
                if raw_line.strip():
                    last_error = raw_line.strip()
                continue
            if not isinstance(payload, dict):
                continue
            message_type = str(payload.get("type", ""))
            if message_type == "state":
                self.gamepad_messages.put(("state", generation, payload))
            elif message_type == "error":
                last_error = str(payload.get("message", "gamepad helper failed"))
        return_code = process.wait()
        self.gamepad_messages.put(("stream_exit", generation, (return_code, last_error)))

    def gamepad_tick(self) -> None:
        latest_state: Optional[Dict[str, object]] = None
        while True:
            try:
                message_type, generation, payload = self.gamepad_messages.get_nowait()
            except queue.Empty:
                break
            if message_type == "devices":
                self._apply_detected_gamepads(payload)
            elif message_type == "detect_error":
                self.gamepad_detecting = False
                self.gamepad_start_after_detect = False
                self.gamepad_status_var.set(f"Gamepad detection failed: {payload}")
            elif message_type == "state" and generation == self.gamepad_generation:
                if isinstance(payload, dict):
                    latest_state = payload
            elif message_type == "stream_exit" and generation == self.gamepad_generation:
                return_code, detail = payload  # type: ignore[misc]
                if self.gamepad_enabled:
                    self.gamepad_enabled = False
                    self.gamepad_process = None
                    self.send("joy disconnect")
                    suffix = f": {detail}" if detail else f" (code {return_code})"
                    self.gamepad_status_var.set("Gamepad input stopped" + suffix)
        if latest_state is not None and self.gamepad_enabled:
            self._apply_gamepad_state(latest_state)
        poll_ms = clamp_int(self._gamepad_int(self.gamepad_poll_var, 20), 10, 200)
        self.root.after(poll_ms, self.gamepad_tick)

    def _apply_gamepad_state(self, snapshot: Dict[str, object]) -> None:
        raw_axes = snapshot.get("axes", [])
        raw_buttons = snapshot.get("buttons", [])
        raw_hats = snapshot.get("hats", [])
        if not isinstance(raw_axes, list) or not isinstance(raw_buttons, list) or not isinstance(raw_hats, list):
            return
        self.gamepad_latest_axes = [
            float(value) if isinstance(value, (int, float)) else 0.0
            for value in raw_axes
        ]

        delta = clamp_int(self._gamepad_int(self.gamepad_delta_var, 20), 1, 300)
        dead = clamp_int(self._gamepad_int(self.gamepad_dead_var, 50), 0, 400)
        for row in range(len(GAMEPAD_AXIS_DEFAULTS)):
            joint = self._gamepad_int(self.gamepad_axis_joint_vars[row], -1)
            axis = self._gamepad_int(self.gamepad_axis_id_vars[row], -1)
            if not 1 <= joint <= 6 or not 0 <= axis < len(raw_axes):
                continue
            try:
                value = scale_gamepad_axis(
                    float(raw_axes[axis]),
                    bool(self.gamepad_axis_invert_vars[row].get()),
                    dead,
                )
            except (TypeError, ValueError):
                continue
            key = f"j{joint}"
            previous = self.gamepad_last_axis.get(key)
            if (
                previous is None
                or abs(value - previous) >= delta
                or (previous != 0 and value == 0)
            ):
                self.gamepad_last_axis[key] = value
                self.gamepad_last_axis_sent_at[key] = time.monotonic()
                self.send(f"joy axis {joint} {value}")

        hat_pressed = [False, False, False, False]
        for raw_hat in raw_hats:
            if not isinstance(raw_hat, list) or len(raw_hat) < 2:
                continue
            try:
                x_value, y_value = int(raw_hat[0]), int(raw_hat[1])
            except (TypeError, ValueError):
                continue
            hat_pressed[0] = hat_pressed[0] or y_value > 0
            hat_pressed[1] = hat_pressed[1] or y_value < 0
            hat_pressed[2] = hat_pressed[2] or x_value < 0
            hat_pressed[3] = hat_pressed[3] or x_value > 0

        for row in range(len(GAMEPAD_BUTTON_DEFAULTS)):
            button = self._gamepad_int(self.gamepad_button_id_vars[row], -1)
            joy_id = self._gamepad_int(self.gamepad_joy_id_vars[row], -1)
            if not 0 <= joy_id <= 32767:
                continue
            direct_pressed = False
            if 0 <= button < len(raw_buttons):
                direct_pressed = bool(raw_buttons[button])
            pressed = 1 if direct_pressed or hat_pressed[row] else 0
            key = f"b{button}->{joy_id}"
            previous = self.gamepad_last_buttons.get(key, 0)
            if pressed != previous:
                self.gamepad_last_buttons[key] = pressed
                self.send(f"joy button {joy_id} {pressed}")

        axes_text = "  ".join(
            f"{index}:{float(value):.2f}" for index, value in enumerate(raw_axes)
            if isinstance(value, (int, float))
        )
        pressed_buttons = ", ".join(
            str(index) for index, value in enumerate(raw_buttons) if bool(value)
        )
        hats_text = " ".join(
            f"{index}:{tuple(value)}" for index, value in enumerate(raw_hats)
            if isinstance(value, list)
        )
        device_name = str(snapshot.get("name", "Gamepad"))
        status = f"Gamepad TX: running - {device_name}"
        self.gamepad_status_var.set(status)
        self.gamepad_debug_var.set(
            f"axes {axes_text or 'none'}\n"
            f"pressed buttons {pressed_buttons or 'none'}\n"
            f"hats {hats_text or 'none'}\n"
            "Servo control is available only from the Drill servo panel."
        )

    def _stop_gamepad_process(self) -> None:
        self.gamepad_generation += 1
        process = self.gamepad_process
        self.gamepad_process = None
        if process is None or process.poll() is not None:
            return
        try:
            process.terminate()
            process.wait(timeout=1)
        except (OSError, subprocess.TimeoutExpired):
            try:
                process.kill()
            except OSError:
                pass

    def stop_gamepad_mode(self) -> None:
        self.gamepad_enabled = False
        self.gamepad_latest_axes = []
        self._stop_gamepad_process()
        self.gamepad_status_var.set("Gamepad TX: paused, safety stop sent")
        self.send("joy disconnect")

    def on_close(self) -> None:
        self.gamepad_enabled = False
        self._stop_gamepad_process()
        if self.link.is_open():
            try:
                self.send("failsafe linkloss")
            finally:
                self.link.close()
        self.root.destroy()


WEB_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>F401 Test GUI</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f5f7fa;
      --panel: #ffffff;
      --line: #d7dde5;
      --text: #17202a;
      --muted: #627386;
      --blue: #1b66c9;
      --red: #bd1e34;
      --green: #14805e;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: Segoe UI, Arial, sans-serif;
      font-size: 14px;
    }
    header {
      display: flex;
      align-items: center;
      gap: 8px;
      padding: 10px 12px;
      background: var(--panel);
      border-bottom: 1px solid var(--line);
      position: sticky;
      top: 0;
      z-index: 3;
    }
    main { padding: 10px; }
    .status { margin-left: auto; color: var(--muted); display: flex; gap: 14px; }
    .tabs { display: flex; flex-wrap: wrap; gap: 6px; margin-bottom: 10px; }
    .tab-btn {
      border: 1px solid var(--line);
      background: var(--panel);
      padding: 7px 10px;
      border-radius: 6px;
      cursor: pointer;
    }
    .tab-btn.active { border-color: var(--blue); color: var(--blue); font-weight: 600; }
    .tab { display: none; }
    .tab.active { display: block; }
    .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 10px;
      margin-bottom: 10px;
    }
    .grid { display: grid; gap: 8px; }
    .grid.cols2 { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    .grid.cols3 { grid-template-columns: repeat(3, minmax(0, 1fr)); }
    .row { display: flex; align-items: center; gap: 6px; flex-wrap: wrap; margin: 4px 0; }
    .axis-row {
      display: grid;
      grid-template-columns: 130px 70px 210px minmax(220px, 1fr) 145px;
      gap: 8px;
      align-items: center;
      padding: 6px 0;
      border-bottom: 1px solid #edf1f5;
    }
    .axis-row:last-child { border-bottom: 0; }
    button {
      border: 1px solid var(--line);
      background: #fff;
      color: var(--text);
      padding: 6px 9px;
      border-radius: 6px;
      cursor: pointer;
    }
    button:hover { border-color: var(--blue); }
    button.primary { background: var(--blue); color: #fff; border-color: var(--blue); }
    button.danger { background: var(--red); color: #fff; border-color: var(--red); }
    button.green { background: var(--green); color: #fff; border-color: var(--green); }
    input, select {
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 6px 7px;
      min-height: 31px;
    }
    input[type=number] { width: 84px; }
    input[type=range] { width: 100%; }
    label { color: var(--muted); }
    .small { color: var(--muted); font-size: 12px; }
    .mono { font-family: Consolas, monospace; }
    h2 {
      font-size: 16px;
      margin: 0 0 8px;
    }
    #log {
      height: 330px;
      overflow: auto;
      background: #0f1720;
      color: #d7e7ff;
      border-radius: 8px;
      padding: 8px;
      font-family: Consolas, monospace;
      white-space: pre-wrap;
    }
    #cmd { flex: 1; min-width: 240px; }
    .status-table {
      width: 100%;
      border-collapse: collapse;
      margin-top: 8px;
    }
    .status-table th,
    .status-table td {
      text-align: left;
      padding: 5px 6px;
      border-bottom: 1px solid #edf1f5;
      font-variant-numeric: tabular-nums;
    }
    .status-table th { color: var(--muted); font-weight: 600; }
    @media (max-width: 900px) {
      .axis-row { grid-template-columns: 1fr; }
      .grid.cols2, .grid.cols3 { grid-template-columns: 1fr; }
      header { align-items: stretch; flex-direction: column; }
      .status { margin-left: 0; }
    }
  </style>
</head>
<body>
  <header>
    <label>Port <select id="port"></select></label>
    <button onclick="refreshPorts()">Refresh</button>
    <label>Baud <input id="baud" type="number" value="115200"></label>
    <label><input id="dry" type="checkbox"> Dry run</label>
    <button class="primary" onclick="connect()">Connect</button>
    <button onclick="disconnect()">Disconnect</button>
    <button class="danger" onclick="send('stopall')">STOP ALL</button>
    <div class="status">
      <span id="connected">Disconnected</span>
      <span id="mode">Mode: unknown</span>
      <span id="fault">Fault: unknown</span>
    </div>
  </header>
  <main>
    <div class="tabs" id="tabs"></div>

    <section id="console" class="tab active">
      <div class="panel">
        <h2>Quick commands</h2>
        <div class="row">
          <button onclick="send('mode safe')">SAFE</button>
           <button onclick="send('mode arm confirm')">ARM</button>
           <button onclick="send('mode armtest confirm')">ARMTEST</button>
           <button onclick="send('mode drill confirm')">DRILL</button>
          <button onclick="send('stop')">Stop</button>
          <button class="danger" onclick="send('stopall')">StopAll</button>
        </div>
        <div class="row">
          <button onclick="send('help')">Help</button>
          <button onclick="send('params')">Params</button>
          <button onclick="send('get mode')">Get mode</button>
          <button onclick="send('get fault')">Get fault</button>
          <button onclick="send('get motors')">Get motors</button>
          <button onclick="send('get sensors')">Get sensors</button>
          <button onclick="send('get as5600')">Get AS5600</button>
          <button onclick="send('stream off')">Stream off</button>
        </div>
        <div class="row">
          <button onclick="send('save')">Save</button>
          <button onclick="many(['stopall','mode safe','save'])">Safe + StopAll + Save</button>
          <button onclick="factoryReset()">Factory reset</button>
          <button onclick="send('quit')">Quit firmware</button>
        </div>
      </div>
      <div class="panel">
        <h2>Live F401 status</h2>
        <div class="row small mono">
          <span id="liveMode">Mode: -</span>
          <span id="liveStopmode">Stopmode: -</span>
          <span id="liveSeen">Live status: waiting</span>
        </div>
        <table class="status-table">
          <thead>
            <tr><th>Joint</th><th>PWM</th><th>DIR</th><th>EN</th><th>Angle</th><th>Vel dps</th></tr>
          </thead>
          <tbody id="liveStatusRows"></tbody>
        </table>
      </div>
      <div class="panel">
        <h2>Serial monitor</h2>
        <div id="log"></div>
        <div class="row" style="margin-top:8px">
          <input id="cmd" placeholder="Type any F401 command, for example: params" onkeydown="if(event.key==='Enter') sendManual()">
          <button onclick="sendManual()">Send</button>
          <button onclick="clearLog()">Clear log</button>
        </div>
        <div class="row">
          <label>seq id <input id="seqid" type="number" value="1"></label>
          <input id="seqcmd" value="params" style="flex:1; min-width:220px">
          <button onclick="sendSeq()">Send seq</button>
        </div>
      </div>
    </section>

    <section id="arm" class="tab">
      <div class="panel">
        <h2>ARM drive</h2>
        <div class="row">
          <button onclick="send('mode arm confirm')">Mode ARM</button>
          <button onclick="send('mode armtest confirm')">Mode ARMTEST</button>
          <button onclick="send('stop')">Stop</button>
          <button class="danger" onclick="send('stopall')">StopAll</button>
          <label><input id="liveaxis" type="checkbox"> Live axis sliders</label>
        </div>
        <div id="axisRows"></div>
      </div>
    </section>

    <section id="position" class="tab">
      <div class="panel">
        <h2>Sensors</h2>
        <div class="row">
          <button onclick="send('get sensors')">Get sensors</button>
          <button onclick="send('get as5600')">Get all AS5600</button>
          <label>CH <input id="as5600ch" type="number" min="0" max="7" value="0"></label>
          <button onclick="send('get as5600 '+v('as5600ch'))">Get CH</button>
          <label>Stream CH <input id="streamch" type="number" min="0" max="7" value="0"></label>
          <label>ms <input id="streamms" type="number" min="50" max="5000" value="250"></label>
          <button onclick="send('stream as5600 '+v('streamch')+' '+v('streamms'))">Start stream</button>
          <button onclick="send('stream off')">Stream off</button>
        </div>
      </div>
      <div id="positionRows"></div>
    </section>

    <section id="tuning" class="tab">
      <div class="panel">
        <h2>General tuning</h2>
        <div class="row">
          <label>Joint <input id="tuneAxis" type="number" min="1" max="6" value="1" onchange="loadBpAxis()"></label>
          <button onclick="send('params')">Params</button>
          <button onclick="send('get motors')">Get motors</button>
          <button onclick="send('get fault')">Get fault</button>
        </div>
        <div class="grid cols2" id="tuneRows"></div>
      </div>
      <div class="panel">
        <h2>Keybinds and sensor mapping</h2>
        <div class="row">
          <label>KEYFWD <input id="keyfwd" type="number" value="-1"></label>
          <button onclick="setTune('keybind', v('keyfwd'))">Set keyfwd</button>
          <label>KEYBACK <input id="keyback" type="number" value="-1"></label>
          <button onclick="setTune('negkeybind', v('keyback'))">Set keyback</button>
          <label>AS5600 CH <input id="tuneAs" type="number" min="-1" max="7" value="0"></label>
          <button onclick="setTune('as5600', v('tuneAs'))">Set as5600</button>
        </div>
      </div>
      <div class="panel">
        <h2>J1-J3 multi-breakpoint boundaries</h2>
        <div id="bpSlotRows"></div>
        <div class="small">Configuration changes require a stopped joint.</div>
      </div>
      <div class="panel">
        <h2>3-layer brakepoint model (J1-J3 only)</h2>
        <div class="grid cols2">
          <div class="row"><label>Reaction (ms) <input id="bpReact" type="number" min="0" max="500" value="40"></label><button onclick="setBpTune('bpreact','bpReact')">Set bpreact</button></div>
          <div class="row"><label>Decel (deg/s²) <input id="bpDecel" type="number" min="50" max="20000" value="500"></label><button onclick="setBpTune('bpdecel','bpDecel')">Set bpdecel</button></div>
          <div class="row"><label>Lead margin (deg) <input id="bpLead" type="number" min="0.1" max="20" step="0.1" value="0.60"></label><button onclick="setBpTune('bplead','bpLead')">Set bplead</button></div>
          <div class="row"><label>Recovery clearance (deg) <input id="bpClear" type="number" min="0.2" max="20" step="0.1" value="0.80"></label><button onclick="setBpTune('bpclear','bpClear')">Set bpclear</button></div>
          <div class="row"><label>Recovery max PWM <input id="bpRecPwm" type="number" min="0" max="255" value="70"></label><button onclick="setBpTune('bprecpwm','bpRecPwm')">Set bprecpwm</button></div>
          <div class="row"><label>Recovery timeout (ms) <input id="bpRecTime" type="number" min="250" max="10000" value="2500"></label><button onclick="setBpTune('bprectime','bpRecTime')">Set bprectime</button></div>
        </div>
        <div class="small">Early-brake lead = speed × reaction + speed² / (2 × decel) + lead margin</div>
        <div class="row">
          <span id="bpStatus" class="small">J1 firmware defaults shown; Params loads live values.</span>
          <button onclick="setAllBpTuning()">Set all BP tuning</button>
          <button onclick="send('params')">Params / load</button>
        </div>
      </div>
    </section>

    <section id="drill" class="tab">
      <div class="panel">
        <h2>DRILL mode</h2>
        <div class="row">
          <button onclick="send('mode drill confirm')">Mode DRILL</button>
          <button onclick="send('mode safe')">Mode SAFE</button>
          <button class="danger" onclick="send('stopall')">StopAll</button>
        </div>
      </div>
      <div class="grid cols2">
        <div class="panel">
          <h2>Elevator BTS4+BTS5</h2>
          <div class="row"><label>PWM <input id="elvpwm" type="range" min="0" max="255" value="80" oninput="syncText('elvpwm','elvpwmn')"></label><input id="elvpwmn" type="number" value="80" oninput="syncText('elvpwmn','elvpwm')"></div>
          <div class="row"><button onclick="send('elv up '+v('elvpwm'))">Up</button><button onclick="send('elv down '+v('elvpwm'))">Down</button><button onclick="send('elv stop')">Stop</button></div>
          <div class="row"><label>default <input id="elvdef" type="number" value="100"></label><button onclick="send('elv default '+v('elvdef'))">Set default</button></div>
          <div class="row"><label>maxpwm <input id="elvmax" type="number" value="255"></label><button onclick="send('elv maxpwm '+v('elvmax'))">Set maxpwm</button></div>
          <div class="row"><label>invert <input id="elvinv" type="number" min="0" max="1" value="0"></label><button onclick="send('elv invert '+v('elvinv'))">Set invert</button></div>
          <div class="row"><label>BTS4 sendpwm <input id="elvsp4" type="number" min="0" max="1" step="0.01" value="1.00"></label><button onclick="setElevatorSendPwm(4,'elvsp4')">Set BTS4</button></div>
          <div class="row"><label>BTS5 sendpwm <input id="elvsp5" type="number" min="0" max="1" step="0.01" value="1.00"></label><button onclick="setElevatorSendPwm(5,'elvsp5')">Set BTS5</button></div>
          <div class="row"><button onclick="send('params')">Params / load</button></div>
        </div>
        <div class="panel">
          <h2>Drill BTS6</h2>
          <div class="row"><label>PWM <input id="drillpwm" type="range" min="0" max="255" value="80" oninput="syncText('drillpwm','drillpwmn')"></label><input id="drillpwmn" type="number" value="80" oninput="syncText('drillpwmn','drillpwm')"></div>
          <div class="row"><button onclick="send('drill dig '+v('drillpwm'))">Dig</button><button onclick="send('drill extract '+v('drillpwm'))">Extract</button><button onclick="send('drill stop')">Stop</button></div>
          <div class="row"><label>default <input id="drilldef" type="number" value="100"></label><button onclick="send('drill default '+v('drilldef'))">Set default</button></div>
          <div class="row"><label>maxpwm <input id="drillmax" type="number" value="255"></label><button onclick="send('drill maxpwm '+v('drillmax'))">Set maxpwm</button></div>
          <div class="row"><label>invert <input id="drillinv" type="number" min="0" max="1" value="0"></label><button onclick="send('drill invert '+v('drillinv'))">Set invert</button></div>
        </div>
      </div>
      <div class="panel">
        <h2>Power HD 1235MG tube selector</h2>
        <div class="row">
          <strong id="servoState">Servo state: not initialized</strong>
          <span id="servoMotion">Servo: idle</span>
          <span id="servoAngleStatus">Estimated PWM angle: unavailable</span>
          <span id="servoSafety">Safety: BOOT</span>
        </div>
        <div class="row">
          <strong id="servoReference">Reference: invalid</strong>
          <span id="servoManual">State/fixed moves: speed-controlled ramp | step buttons: one click</span>
          <span id="servoError">No servo error</span>
        </div>
        <div class="row">
          <button onclick="send('servo 1')">Final lid</button>
          <button onclick="send('servo 2')">Tube 4</button>
          <button onclick="send('servo 3')">Tube 3</button>
          <button onclick="send('servo 4')">Tube 2</button>
          <button onclick="send('servo 5')">Tube 1</button>
          <button onclick="send('servowiggle')">Servo Wiggle</button>
        </div>
        <div class="row">
          <button onclick="send('servo pulse 1000')">1000 us</button>
          <button onclick="send('servo pulse 1500')">1500 us</button>
          <button onclick="send('servo pulse 2000')">2000 us</button>
          <button onclick="send('servo pulse off')">PWM off</button>
          <button onclick="send('servo diag')">Timer diag</button>
        </div>
        <div class="row">
          <strong id="servoCalibration">Servo calibration: none saved</strong>
          <button onclick="send('set servostate 1')">Save current as Final lid</button>
          <button onclick="send('set servostate 2')">Save current as Tube 4</button>
          <button onclick="send('set servostate 3')">Save current as Tube 3</button>
          <button onclick="send('set servostate 4')">Save current as Tube 2</button>
          <button onclick="send('set servostate 5')">Save current as Tube 1</button>
          <button onclick="send('set servocal clear')">Clear calibration</button>
          <button class="primary" onclick="send('save')">Save calibration to flash</button>
        </div>
        <div class="small">State and custom-pulse buttons use the configured speed ramp and finish at the exact target. Forward/Backward performs one small step per click; there is no hold-repeat loop.</div>
        <div class="row">
          <label>Custom pulse (us) <input id="servoPulse" type="number" min="800" max="2200" value="1500"></label>
          <button onclick="send('servo pulse '+v('servoPulse'))">Apply once</button>
          <label>Manual step (us) <input id="servoStep" type="number" min="1" max="100" value="10"></label>
          <button onclick="send('servo backward '+v('servoStep'))">Backward</button>
          <button onclick="send('servo forward '+v('servoStep'))">Forward</button>
          <label>Wiggle angle <input id="servoWiggleAngle" type="number" min="1" max="30" value="5"></label>
          <button onclick="send('set servowiggleangle '+v('servoWiggleAngle'))">Set</button>
          <label>Invert <input id="servoInvert" type="number" min="0" max="1" value="0"></label>
          <button onclick="send('set servo invert '+v('servoInvert'))">Set invert</button>
          <label>Speed (deg/s) <input id="servoSpeed" type="number" min="5" max="120" value="30"></label>
          <button onclick="send('set servospeed '+v('servoSpeed'))">Set speed</button>
        </div>
      </div>
      <div class="panel">
        <h2>Drill LEDs / active-LOW relay module</h2>
        <div class="grid cols2">
          <div class="row"><strong id="drillLed1">Drill LED 1: -</strong><button onclick="send('DrillLed 1 1')">ON</button><button onclick="send('DrillLed 1 0')">OFF</button></div>
          <div class="row"><strong id="drillLed2">Drill LED 2: -</strong><button onclick="send('DrillLed 2 1')">ON</button><button onclick="send('DrillLed 2 0')">OFF</button></div>
          <div class="row"><strong id="drillLed3">Drill LED 3: -</strong><button onclick="send('DrillLed 3 1')">ON</button><button onclick="send('DrillLed 3 0')">OFF</button></div>
          <div class="row"><strong id="drillLed4">Drill LED 4: -</strong><button onclick="send('DrillLed 4 1')">ON</button><button onclick="send('DrillLed 4 0')">OFF</button></div>
        </div>
      </div>
    </section>

    <section id="joy" class="tab">
      <div class="panel">
        <h2>Joystick/keybind command test</h2>
        <div class="row">
          <label>Target (J1-J6) <input id="joyAxis" type="number" min="1" max="6" value="1"></label>
          <label style="flex:1">Value <input id="joyValue" type="range" min="-1000" max="1000" value="0" oninput="syncText('joyValue','joyValueN')"></label>
          <input id="joyValueN" type="number" value="0" oninput="syncText('joyValueN','joyValue')">
          <button onclick="send('joy axis '+v('joyAxis')+' '+v('joyValue'))">Send joy axis</button>
          <button onclick="setBoth('joyValue','joyValueN',0); send('joy axis '+v('joyAxis')+' 0')">Zero + send</button>
        </div>
        <div class="row">
          <label>Button id <input id="joyButton" type="number" value="1000"></label>
          <button onclick="send('joy button '+v('joyButton')+' 1')">Press</button>
          <button onclick="send('joy button '+v('joyButton')+' 0')">Release</button>
        </div>
        <div class="row">
          <button onclick="send('joy button 1000 1')">Hat up press</button><button onclick="send('joy button 1000 0')">release</button>
          <button onclick="send('joy button 1001 1')">Hat down press</button><button onclick="send('joy button 1001 0')">release</button>
          <button onclick="send('joy button 1002 1')">Hat left press</button><button onclick="send('joy button 1002 0')">release</button>
          <button onclick="send('joy button 1003 1')">Hat right press</button><button onclick="send('joy button 1003 0')">release</button>
        </div>
      </div>
    </section>

    <section id="gamepad" class="tab">
      <div class="panel">
        <h2>Gamepad ARM / DRILL control</h2>
        <div class="row">
          <button class="primary" onclick="startGamepadMode()">Enable gamepad TX</button>
          <button onclick="stopGamepadMode()">Pause gamepad TX</button>
          <button onclick="detectGamepads()">Detect gamepads</button>
          <button onclick="send('mode arm confirm')">Mode ARM</button>
          <button onclick="send('mode drill confirm')">Mode DRILL</button>
          <button onclick="send('stop')">Stop ARM</button>
          <button class="danger" onclick="send('stopall')">StopAll</button>
        </div>
        <div class="row">
          <label>Gamepad <select id="gamepadSelect"></select></label>
          <label>Axis dead <input id="gpDead" type="number" min="0" max="400" value="50"></label>
          <label>Axis delta <input id="gpDelta" type="number" min="1" max="300" value="20"></label>
          <label>Poll ms <input id="gpPoll" type="number" min="10" max="200" value="20"></label>
        </div>
        <div class="small">
          Default browser mapping: left stick X -> J1 base, left stick Y -> J2 shoulder,
          right stick Y -> J3 elbow, right stick X -> J5 twist, and gamepad axis 2
          Servo control is intentionally excluded. Select Mode ARM explicitly before transmitting.
        </div>
        <div id="gamepadStatus" class="small mono" style="margin-top:8px">Gamepad TX: paused</div>
      </div>
      <div class="panel">
        <h2>Axis mapping</h2>
        <div id="gamepadAxisRows"></div>
      </div>
      <div class="panel">
        <h2>Button mapping</h2>
        <div class="small">Browser buttons 12/13/14/15 are commonly D-pad up/down/left/right.</div>
        <div id="gamepadButtonRows"></div>
      </div>
      <div class="panel">
        <h2>Live gamepad values</h2>
        <pre id="gamepadDebug" class="mono small">Press a gamepad button once, then click Detect gamepads.</pre>
      </div>
    </section>
  </main>
<script>
const AXES = ['J1 Base','J2 Shoulder','J3 Elbow','J4 Pitch','J5 Twist','J6 Gripper'];
const STATUS_FIELDS = ['PWM','DIR','EN','ANGLE','VEL_DPS'];
const BP_DEFAULTS = {
  1:{BPREACT_MS:'40',BPDECEL_DPS2:'500',BPLEAD_DEG:'0.60',BPCLEAR_DEG:'0.80',BPRECPWM:'70',BPRECTIME_MS:'2500'},
  2:{BPREACT_MS:'40',BPDECEL_DPS2:'350',BPLEAD_DEG:'0.80',BPCLEAR_DEG:'1.00',BPRECPWM:'90',BPRECTIME_MS:'3500'},
  3:{BPREACT_MS:'40',BPDECEL_DPS2:'650',BPLEAD_DEG:'0.60',BPCLEAR_DEG:'0.80',BPRECPWM:'65',BPRECTIME_MS:'2500'}
};
const BP_INPUTS = {
  BPREACT_MS:'bpReact', BPDECEL_DPS2:'bpDecel', BPLEAD_DEG:'bpLead',
  BPCLEAR_DEG:'bpClear', BPRECPWM:'bpRecPwm', BPRECTIME_MS:'bpRecTime'
};
const BP_COMMAND_FIELDS = {
  bpreact:'BPREACT_MS', bpdecel:'BPDECEL_DPS2', bplead:'BPLEAD_DEG',
  bpclear:'BPCLEAR_DEG', bprecpwm:'BPRECPWM', bprectime:'BPRECTIME_MS'
};
const BP_MODES = ['OFF','WALL','BLOCK_INCREASE','BLOCK_DECREASE'];
const MAX_BROWSER_LOG_LINES = 1200;
const tabs = [['console','Console'],['arm','ARM Drive'],['position','Position/PID'],['tuning','Tuning'],['drill','DRILL'],['joy','Joy/Keybind'],['gamepad','Gamepad ARM']];
let since = 0;
let gamepadEnabled = false;
let gamepadTimer = null;
let gamepadLastAxis = {};
let gamepadLastAxisSentAt = {};
let gamepadLastButtons = {};
let liveStatusRevision = -1;
let bpTuningRevision = -1;
let bpTuningAxes = {};
let bpConfigRevision = -1;
let bpConfigAxes = {};
let elevatorSendPwmRevision = -1;
let pollFailures = 0;

function q(id) { return document.getElementById(id); }
function v(id) { return q(id).value; }
function setText(id, value) {
  const node = q(id);
  if (node && node.textContent !== value) node.textContent = value;
}
function setBoth(a, b, value) { q(a).value = value; q(b).value = value; }
function syncText(source, target) { q(target).value = q(source).value; }
function tuneAxis() { return Math.max(1, Math.min(6, parseInt(v('tuneAxis') || '1'))); }
function setTune(name, value) { send(`set ${tuneAxis()} ${name} ${value}`); }
function setElevatorSendPwm(bts, input) {
  const raw = v(input);
  const value = Number(raw);
  const scaled = value * 100;
  const rounded = Math.round(scaled);
  if (!Number.isFinite(value) || value < 0 || value > 1 || Math.abs(scaled - rounded) > 1e-6) {
    alert('sendpwm must be between 0.00 and 1.00 with 0.01 precision.');
    return;
  }
  const normalized = (rounded / 100).toFixed(2);
  q(input).value = normalized;
  send(`set bts${bts} sendpwm ${normalized}`);
}
function updateElevatorSendPwm(snapshot) {
  if (!snapshot) return;
  const revision = Number(snapshot.revision || 0);
  if (revision === elevatorSendPwmRevision) return;
  elevatorSendPwmRevision = revision;
  const values = snapshot.values || {};
  if (values.SENDPWM_BTS4 !== undefined) q('elvsp4').value = values.SENDPWM_BTS4;
  if (values.SENDPWM_BTS5 !== undefined) q('elvsp5').value = values.SENDPWM_BTS5;
}
function selectedBpAxis() {
  const axis = tuneAxis();
  if (axis <= 3) return axis;
  setText('bpStatus', `J${axis}: BP model settings are unavailable (J1-J3 only).`);
  return null;
}
function loadBpAxis() {
  loadBpTuningAxis();
  loadBpConfigAxis();
}
function loadBpTuningAxis() {
  const axis = selectedBpAxis();
  if (axis === null) return;
  const liveValues = bpTuningAxes[String(axis)] || bpTuningAxes[axis];
  const values = liveValues || BP_DEFAULTS[axis];
  Object.entries(BP_INPUTS).forEach(([field, input]) => { q(input).value = values[field]; });
  setText('bpStatus', `J${axis}: ${liveValues ? 'last Params/local values' : 'firmware defaults'} shown.`);
}
function updateBpTuning(snapshot) {
  if (!snapshot) return;
  const revision = Number(snapshot.revision || 0);
  if (revision === bpTuningRevision) return;
  bpTuningRevision = revision;
  bpTuningAxes = snapshot.axes || {};
  loadBpTuningAxis();
}
function setBpTune(name, input) {
  const axis = selectedBpAxis();
  if (axis === null) return;
  const value = v(input);
  send(`set ${axis} ${name} ${value}`);
  bpTuningAxes[String(axis)] = {...(bpTuningAxes[String(axis)] || BP_DEFAULTS[axis]), [BP_COMMAND_FIELDS[name]]:value};
  setText('bpStatus', `J${axis}: ${name} sent; use Params to verify.`);
}
function setAllBpTuning() {
  const axis = selectedBpAxis();
  if (axis === null) return;
  many([
    `set ${axis} bpreact ${v('bpReact')}`,
    `set ${axis} bpdecel ${v('bpDecel')}`,
    `set ${axis} bplead ${v('bpLead')}`,
    `set ${axis} bpclear ${v('bpClear')}`,
    `set ${axis} bprecpwm ${v('bpRecPwm')}`,
    `set ${axis} bprectime ${v('bpRecTime')}`
  ]);
  const values = {};
  Object.entries(BP_INPUTS).forEach(([field, input]) => { values[field] = v(input); });
  bpTuningAxes[String(axis)] = values;
  setText('bpStatus', `J${axis}: all BP tuning commands sent; use Params to verify.`);
}
function defaultBpConfig() {
  return {
    slots:{
      1:{MODE:'OFF',ANGLE:'NA'}, 2:{MODE:'OFF',ANGLE:'NA'},
      3:{MODE:'OFF',ANGLE:'NA'}, 4:{MODE:'OFF',ANGLE:'NA'}
    }
  };
}
function makeBpSlotRows() {
  const box = q('bpSlotRows');
  for (let slot = 1; slot <= 4; slot++) {
    const row = document.createElement('div');
    row.className = 'row';
    row.innerHTML = `
      <strong>BP${slot}</strong>
      <label>Mode <select id="bpSlotMode${slot}">${BP_MODES.map(mode => `<option>${mode}</option>`).join('')}</select></label>
      <label>Angle <input id="bpSlotAngle${slot}" type="number" min="0" max="359.999" step="0.1" value="0"></label>
      <button onclick="setBpSlot(${slot})">Apply</button>
      <button onclick="disableBpSlot(${slot})">Off</button>`;
    box.appendChild(row);
  }
}
function loadBpConfigAxis() {
  const axis = selectedBpAxis();
  if (axis === null) return;
  const config = bpConfigAxes[String(axis)] || bpConfigAxes[axis] || defaultBpConfig();
  for (let slot = 1; slot <= 4; slot++) {
    const values = config.slots[String(slot)] || config.slots[slot] || {MODE:'OFF',ANGLE:'NA'};
    q(`bpSlotMode${slot}`).value = values.MODE;
    q(`bpSlotAngle${slot}`).value = values.ANGLE === 'NA' ? '0' : values.ANGLE;
  }
}
function updateBpConfig(snapshot) {
  if (!snapshot) return;
  const revision = Number(snapshot.revision || 0);
  if (revision === bpConfigRevision) return;
  bpConfigRevision = revision;
  bpConfigAxes = snapshot.axes || {};
  loadBpConfigAxis();
}
function setBpSlot(slot) {
  const axis = selectedBpAxis();
  if (axis === null) return;
  const mode = v(`bpSlotMode${slot}`);
  if (mode === 'OFF') {
    disableBpSlot(slot);
    return;
  }
  const angle = v(`bpSlotAngle${slot}`);
  send(`set ${axis} bp ${slot} ${mode.toLowerCase()} ${angle}`);
  const config = bpConfigAxes[String(axis)] || defaultBpConfig();
  config.slots[String(slot)] = {MODE:mode,ANGLE:angle};
  bpConfigAxes[String(axis)] = config;
  setText('bpStatus', `J${axis} BP${slot}: configuration sent; Params verifies acceptance.`);
}
function disableBpSlot(slot) {
  const axis = selectedBpAxis();
  if (axis === null) return;
  send(`set ${axis} bp ${slot} off`);
  q(`bpSlotMode${slot}`).value = 'OFF';
  const config = bpConfigAxes[String(axis)] || defaultBpConfig();
  config.slots[String(slot)] = {MODE:'OFF',ANGLE:'NA'};
  bpConfigAxes[String(axis)] = config;
  setText('bpStatus', `J${axis} BP${slot}: disable sent; Params verifies acceptance.`);
}
function makeLiveStatusRows() {
  const box = q('liveStatusRows');
  AXES.forEach((name, index) => {
    const row = document.createElement('tr');
    row.innerHTML = `<td>${name}</td>${STATUS_FIELDS.map(field => `<td id="liveJ${index + 1}_${field}">-</td>`).join('')}`;
    box.appendChild(row);
  });
}
function servoStateLabel(state) {
  const text = String(state);
  return ({
    '1':'Final lid',
    '2':'Tube 4',
    '3':'Tube 3',
    '4':'Tube 2',
    '5':'Tube 1'
  })[text] || text;
}
function updateLiveStatus(status) {
  if (!status || !Array.isArray(status.axes)) return;
  const revision = Number.isFinite(Number(status.revision)) ? Number(status.revision) : 0;
  if (revision === liveStatusRevision) return;
  liveStatusRevision = revision;
  setText('liveMode', 'Mode: ' + (status.mode || '-'));
  setText('liveStopmode', 'Stopmode: ' + (status.stopmode || '-'));
  const receivedAt = Number(status.received_at || 0);
  const seenText = receivedAt > 0 ? new Date(receivedAt * 1000).toLocaleTimeString() : 'waiting';
  setText('liveSeen', 'Live status: ' + seenText);
  status.axes.forEach((axis, index) => {
    STATUS_FIELDS.forEach(field => {
      const cell = q(`liveJ${index + 1}_${field}`);
      const value = axis && axis[field] !== undefined ? axis[field] : '-';
      if (cell && cell.textContent !== value) cell.textContent = value;
    });
  });
  const servo = status.servo || {};
  setText(
    'servoState',
    String(servo.SERVO_INIT || '0') === '1'
      ? 'Servo state: ' + servoStateLabel(servo.SERVO_STATE || '-') +
        ' → target ' + servoStateLabel(servo.SERVO_TARGET || '-')
      : 'Servo state: not initialized'
  );
  setText('servoMotion', String(servo.SERVO_MOVING || '0') === '1' ? 'Servo: moving' : 'Servo: idle');
  setText(
    'servoSafety',
    `Safety: ${String(servo.SERVO_SAFETY || '-')}`
  );
  const referenceValid = String(servo.SERVO_REFERENCE_VALID || '0') === '1';
  const servoError = String(servo.SERVO_ERROR || 'NONE');
  const currentPulse = String(servo.SERVO_CURRENT_PULSE || 'NA');
  let referenceText;
  if (servoError === 'TARGET_PULSE_RANGE') {
    referenceText = `Calibration rejected at ${currentPulse} us: tube pulses must remain in state order`;
  } else if (servoError === 'TARGET_NOT_CALIBRATED') {
    referenceText = 'Target tube is not calibrated; apply a fixed pulse and save it first';
  } else if (referenceValid) {
    referenceText = `Current state reference: ready at ${String(servo.SERVO_REFERENCE_PULSE || currentPulse)} us`;
  } else {
    referenceText = `Current state: not declared | pulse: ${currentPulse} us`;
  }
  setText(
    'servoReference',
    referenceText
  );
  setText(
    'servoManual',
    'State/fixed moves: speed-controlled ramp | step buttons: one click'
  );
  const errorText = servoError === 'TARGET_PULSE_RANGE'
    ? 'Calibrated tube pulses must remain in logical state order'
    : servoError === 'TARGET_NOT_CALIBRATED'
      ? 'Target tube must be calibrated before automatic selection'
    : servoError === 'REFERENCE_REQUIRED'
      ? 'Calibrate or declare a current tube before selecting a target'
      : servoError === 'NONE'
          ? 'No servo error'
          : `Servo warning: ${servoError}`;
  setText('servoError', errorText);
  const currentAngle = String(servo.SERVO_CURRENT_ANGLE || 'NA');
  const targetAngle = String(servo.SERVO_TARGET_ANGLE || 'NA');
  const referenceAngle = String(servo.SERVO_REFERENCE_ANGLE || 'NA');
  setText(
    'servoAngleStatus',
    currentAngle === 'NA'
      ? 'Estimated PWM angle: unavailable'
      : `Estimated PWM angle: ${currentAngle}° | target: ${targetAngle}° | calibration reference: ${referenceAngle}°`
  );
  const calPulses = [1,2,3,4,5].map(
    state => String(servo[`SERVO_CAL_P${state}`] || 'NA')
  );
  const calSaved = calPulses.filter(value => value !== 'NA').length;
  setText(
    'servoCalibration',
    `Servo calibration: ${calSaved}/5 saved | ` +
    calPulses.map((value, index) => `${servoStateLabel(index + 1)}=${value} us`).join(' | ')
  );
  if (q('servoAngle') && servo.SERVOANGLE !== undefined) q('servoAngle').value = servo.SERVOANGLE;
  if (q('servoWiggleAngle') && servo.SERVOWIGGLEANGLE !== undefined) q('servoWiggleAngle').value = servo.SERVOWIGGLEANGLE;
  if (q('servoInvert') && servo.SERVO_INVERT !== undefined) q('servoInvert').value = servo.SERVO_INVERT;
  if (q('servoSpeed') && servo.SERVO_CONFIGURED_SPEED !== undefined) {
    q('servoSpeed').value = servo.SERVO_CONFIGURED_SPEED;
  } else if (q('servoSpeed') && servo.SERVOSPEED !== undefined) {
    q('servoSpeed').value = servo.SERVOSPEED;
  }
  const leds = Array.isArray(status.drill_leds) ? status.drill_leds : [];
  leds.slice(0, 4).forEach((value, index) => {
    setText(`drillLed${index + 1}`, `Drill LED ${index + 1}: ${String(value) === '1' ? 'ON' : 'OFF'}`);
  });
}

function appendLogLines(items) {
  if (!items.length) return;
  const log = q('log');
  const fragment = document.createDocumentFragment();
  items.forEach(item => fragment.appendChild(document.createTextNode(item[1] + '\n')));
  log.appendChild(fragment);
  while (log.childNodes.length > MAX_BROWSER_LOG_LINES) log.removeChild(log.firstChild);
  log.scrollTop = log.scrollHeight;
}

function makeTabs() {
  const box = q('tabs');
  tabs.forEach(([id, label], index) => {
    const b = document.createElement('button');
    b.className = 'tab-btn' + (index === 0 ? ' active' : '');
    b.textContent = label;
    b.onclick = () => showTab(id);
    box.appendChild(b);
  });
}
function showTab(id) {
  document.querySelectorAll('.tab').forEach(t => t.classList.toggle('active', t.id === id));
  document.querySelectorAll('.tab-btn').forEach((b, i) => b.classList.toggle('active', tabs[i][0] === id));
}
function makeAxisRows() {
  const box = q('axisRows');
  AXES.forEach((name, idx) => {
    const axis = idx + 1;
    const row = document.createElement('div');
    row.className = 'axis-row';
    row.innerHTML = `
      <strong>${name}</strong>
      <input id="pwm${axis}" type="number" min="0" max="255" value="80">
      <span>
        <button onclick="send('forward ${axis} '+v('pwm${axis}'))">Fwd</button>
        <button onclick="send('backward ${axis} '+v('pwm${axis}'))">Back</button>
        <button onclick="send('stop ${axis}')">Stop</button>
      </span>
      <input id="axis${axis}" type="range" min="-1000" max="1000" value="0" oninput="axisSlide(${axis})" onchange="sendAxis(${axis})">
      <span>
        <button onclick="sendAxis(${axis})">Send</button>
        <button onclick="q('axis${axis}').value=0; send('axis ${axis} 0'); send('stop ${axis}')">0+Stop</button>
      </span>`;
    box.appendChild(row);
  });
}
let axisLast = Array(6).fill(0);
let axisAt = Array(6).fill(0);
function axisSlide(axis) {
  if (!q('liveaxis').checked) return;
  const value = parseInt(v('axis'+axis));
  const now = Date.now();
  if (now - axisAt[axis-1] < 60 && Math.abs(value - axisLast[axis-1]) < 40) return;
  sendAxis(axis);
}
function sendAxis(axis) {
  const value = parseInt(v('axis'+axis));
  axisLast[axis-1] = value;
  axisAt[axis-1] = Date.now();
  send(`axis ${axis} ${value}`);
}
function makePositionRows() {
  const box = q('positionRows');
  [1,2,3].forEach(axis => {
    const row = document.createElement('div');
    row.className = 'panel';
    row.innerHTML = `
      <h2>${AXES[axis-1]}</h2>
      <div class="row">
        <button onclick="send('zero ${axis}')">Zero</button>
        <label>Angle <input id="goto${axis}" value="0"></label>
        <button onclick="send('goto ${axis} '+v('goto${axis}'))">Goto</button>
        <label>Rotate <input id="rot${axis}" value="10"></label>
        <button onclick="send('rotate ${axis} '+v('rot${axis}'))">Move</button>
        <button onclick="send('stop ${axis}')">Stop</button>
        <button onclick="many(['set ${axis} stopmode hold','stop ${axis}'])">Hold now</button>
      </div>
      <div class="row">
        <label>Stopmode <select id="stopmode${axis}"><option>coast</option><option>brake</option><option selected>hold</option><option>hybrid</option></select></label>
        <button onclick="send('set ${axis} stopmode '+v('stopmode${axis}'))">Set</button>
        <label>KP <input id="kp${axis}" type="number" value="1200"></label>
        <label>KD <input id="kd${axis}" type="number" value="250"></label>
        <button onclick="many(['set ${axis} kp '+v('kp${axis}'),'set ${axis} kd '+v('kd${axis}')])">Set KP/KD</button>
      </div>
      <div class="row">
        <label>MinPWM <input id="minpwm${axis}" type="number" min="0" max="255" value="15"></label>
        <label>Tol deg <input id="tol${axis}" value="0.8"></label>
        <label>AS5600 CH <input id="as${axis}" type="number" min="-1" max="7" value="${axis-1}"></label>
        <button onclick="many(['set ${axis} minpwm '+v('minpwm${axis}'),'set ${axis} tolerance '+v('tol${axis}'),'set ${axis} as5600 '+v('as${axis}')])">Set PID/sensor</button>
        <button class="green" onclick="activeHold(${axis})">Active hold preset</button>
      </div>`;
    box.appendChild(row);
  });
}
function activeHold(axis) {
  many([
    'mode arm confirm',
    `set ${axis} as5600 ${v('as'+axis)}`,
    `set ${axis} stopmode hold`,
    `set ${axis} tolerance ${v('tol'+axis)}`,
    `set ${axis} kp ${v('kp'+axis)}`,
    `set ${axis} kd ${v('kd'+axis)}`,
    `set ${axis} minpwm ${v('minpwm'+axis)}`,
    `stop ${axis}`
  ]);
}
function makeTuneRows() {
  const box = q('tuneRows');
  const items = [
    ['invert',0,0,1], ['maxpwm',255,0,255], ['default',100,0,255],
    ['posgain',1000,100,3000], ['neggain',1000,100,3000], ['dead',50,0,400]
  ];
  items.forEach(([name, val, min, max]) => {
    const row = document.createElement('div');
    row.className = 'row';
    row.innerHTML = `<label>${name} <input id="tune_${name}" type="range" min="${min}" max="${max}" value="${val}" oninput="syncText('tune_${name}','tune_${name}_n')"></label><input id="tune_${name}_n" type="number" min="${min}" max="${max}" value="${val}" oninput="syncText('tune_${name}_n','tune_${name}')"><button onclick="setTune('${name}', v('tune_${name}'))">Set</button>`;
    box.appendChild(row);
  });
}
function makeGamepadRows() {
  const axisDefaults = [
    ['Base', 1, 0, false],
    ['Shoulder', 2, 1, true],
    ['Elbow', 3, 3, true],
    ['Twist', 5, 2, false],
  ];
  const axisBox = q('gamepadAxisRows');
  axisDefaults.forEach((item, index) => {
    const [name, joint, gpAxis, invert] = item;
    const row = document.createElement('div');
    row.className = 'row';
    row.innerHTML = `
      <strong style="width:86px">${name}</strong>
      <label>Target <input id="gpAxisJoint${index}" type="number" min="1" max="6" value="${joint}"></label>
      <label>Gamepad axis <input id="gpAxisId${index}" type="number" min="0" max="15" value="${gpAxis}"></label>
      <label><input id="gpAxisInv${index}" type="checkbox" ${invert ? 'checked' : ''}> Invert</label>
      <button onclick="send('joy axis '+v('gpAxisJoint${index}')+' 0')">Zero this target</button>`;
    axisBox.appendChild(row);
  });
  const buttonDefaults = [
    ['Pitch up', 12, 1000],
    ['Pitch down', 13, 1001],
    ['Gripper left', 14, 1002],
    ['Gripper right', 15, 1003],
  ];
  const buttonBox = q('gamepadButtonRows');
  buttonDefaults.forEach((item, index) => {
    const [name, button, joyId] = item;
    const row = document.createElement('div');
    row.className = 'row';
    row.innerHTML = `
      <strong style="width:100px">${name}</strong>
      <label>Browser button <input id="gpBtnId${index}" type="number" min="0" max="63" value="${button}"></label>
      <label>F401 joy id <input id="gpJoyId${index}" type="number" min="0" max="32767" value="${joyId}"></label>
      <button onclick="send('joy button '+v('gpJoyId${index}')+' 1')">Press test</button>
      <button onclick="send('joy button '+v('gpJoyId${index}')+' 0')">Release test</button>`;
    buttonBox.appendChild(row);
  });
}
async function api(path, data) {
  const options = data === undefined ? {} : {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(data)};
  const res = await fetch(path, options);
  if (!res.ok) throw new Error(await res.text());
  return await res.json();
}
async function send(cmd) {
  if (!cmd || !cmd.trim()) return;
  await api('/api/send', {cmd});
}
async function many(cmds) { await api('/api/send_many', {cmds}); }
function sendManual() {
  const cmd = v('cmd');
  q('cmd').value = '';
  send(cmd);
}
function sendSeq() {
  const id = parseInt(v('seqid') || '1');
  send(`seq ${id} ${v('seqcmd')}`);
  q('seqid').value = id + 1;
}
function factoryReset() {
  if (confirm('Reset RAM settings to firmware defaults?')) send('factoryreset confirm');
}
async function refreshPorts() {
  const data = await api('/api/ports');
  const box = q('port');
  const old = box.value;
  box.innerHTML = '';
  data.ports.forEach(p => {
    const opt = document.createElement('option');
    opt.value = p; opt.textContent = p; box.appendChild(opt);
  });
  if (old) box.value = old;
}
async function connect() {
  await api('/api/connect', {port:v('port'), baud:parseInt(v('baud')), dry_run:q('dry').checked});
}
async function disconnect() { await api('/api/disconnect', {}); }
function detectGamepads() {
  const pads = navigator.getGamepads ? Array.from(navigator.getGamepads()).filter(Boolean) : [];
  const box = q('gamepadSelect');
  const old = box.value;
  box.innerHTML = '';
  pads.forEach(pad => {
    const opt = document.createElement('option');
    opt.value = pad.index;
    opt.textContent = `${pad.index}: ${pad.id}`;
    box.appendChild(opt);
  });
  if (old) box.value = old;
  q('gamepadStatus').textContent = pads.length ? `Gamepads detected: ${pads.length}` : 'No gamepad detected. Press a gamepad button, then Detect.';
}
function selectedGamepad() {
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  const selected = parseInt(v('gamepadSelect') || '0');
  return pads[selected] || Array.from(pads).find(Boolean) || null;
}
function scaledGamepadAxis(raw, invert) {
  let value = invert ? -raw : raw;
  if (value > 1) value = 1;
  if (value < -1) value = -1;
  let scaled = Math.round(value * 1000);
  const dead = Math.max(0, Math.min(400, parseInt(v('gpDead') || '50')));
  if (Math.abs(scaled) <= dead) scaled = 0;
  return scaled;
}
function gamepadAxisMappings() {
  const rows = [];
  for (let i = 0; i < 4; i++) {
    rows.push({
      joint: parseInt(v('gpAxisJoint'+i) || '1'),
      axis: parseInt(v('gpAxisId'+i) || '0'),
      invert: q('gpAxisInv'+i).checked
    });
  }
  return rows;
}
function gamepadButtonMappings() {
  const rows = [];
  for (let i = 0; i < 4; i++) {
    rows.push({
      button: parseInt(v('gpBtnId'+i) || '0'),
      joy: parseInt(v('gpJoyId'+i) || '0')
    });
  }
  return rows;
}
function startGamepadMode() {
  detectGamepads();
  gamepadEnabled = true;
  gamepadLastAxis = {};
  gamepadLastAxisSentAt = {};
  gamepadLastButtons = {};
  q('gamepadStatus').textContent = 'Gamepad TX: running';
  if (gamepadTimer === null) gamepadLoop();
}
function stopGamepadMode() {
  gamepadEnabled = false;
  q('gamepadStatus').textContent = 'Gamepad TX: paused, safety stop sent';
  send('joy disconnect');
}
function gamepadLoop() {
  if (gamepadEnabled) {
    const pad = selectedGamepad();
    if (!pad) {
      gamepadEnabled = false;
      q('gamepadStatus').textContent = 'Gamepad disconnected; safety stop sent';
      send('joy disconnect');
    } else {
      const delta = Math.max(1, parseInt(v('gpDelta') || '20'));
      gamepadAxisMappings().forEach(map => {
        if (!Number.isFinite(map.joint) || map.joint < 1 || map.joint > 6) return;
        if (!Number.isFinite(map.axis) || map.axis < 0 || map.axis >= pad.axes.length) return;
        const value = scaledGamepadAxis(pad.axes[map.axis], map.invert);
        const key = `j${map.joint}`;
        const prev = gamepadLastAxis[key];
        if (
          prev === undefined ||
          Math.abs(value - prev) >= delta ||
          (prev !== 0 && value === 0)
        ) {
          gamepadLastAxis[key] = value;
          gamepadLastAxisSentAt[key] = Date.now();
          send(`joy axis ${map.joint} ${value}`);
        }
      });
      gamepadButtonMappings().forEach(map => {
        if (!Number.isFinite(map.button) || map.button < 0 || map.button >= pad.buttons.length) return;
        const pressed = pad.buttons[map.button].pressed ? 1 : 0;
        const key = `b${map.button}->${map.joy}`;
        const prev = gamepadLastButtons[key] || 0;
        if (pressed !== prev) {
          gamepadLastButtons[key] = pressed;
          send(`joy button ${map.joy} ${pressed}`);
        }
      });
      const axes = pad.axes.map((value, index) => `${index}:${value.toFixed(2)}`).join('  ');
      const buttons = pad.buttons.map((button, index) => button.pressed ? index : null).filter(v => v !== null).join(', ');
      q('gamepadStatus').textContent = `Gamepad TX: running - ${pad.id}`;
      q('gamepadDebug').textContent =
        `axes ${axes}\npressed buttons ${buttons || 'none'}\n` +
        'Servo control is available only from the Drill servo panel.';
    }
  }
  const pollMs = Math.max(10, Math.min(200, parseInt(v('gpPoll') || '20')));
  gamepadTimer = setTimeout(gamepadLoop, pollMs);
}
async function poll() {
  try {
    const data = await api('/api/logs?since='+since);
    pollFailures = 0;
    since = data.next;
    setText('connected', data.connected ? 'Connected' : 'Disconnected');
    setText('mode', 'Mode: ' + data.mode);
    setText('fault', 'Fault: ' + data.fault);
    updateLiveStatus(data.live_status);
    updateBpTuning(data.brakepoint_tuning);
    updateBpConfig(data.brakepoint_config);
    updateElevatorSendPwm(data.elevator_sendpwm);
    appendLogLines(data.lines);
  } catch (err) {
    pollFailures += 1;
    if (pollFailures >= 3) setText('connected', 'Server error');
  }
  setTimeout(poll, 250);
}
function clearLog() { q('log').replaceChildren(); }
async function init() {
  makeTabs();
  makeLiveStatusRows();
  makeAxisRows();
  makePositionRows();
  makeTuneRows();
  makeBpSlotRows();
  makeGamepadRows();
  await refreshPorts();
  const initData = await api('/api/init');
  q('baud').value = initData.baud;
  q('dry').checked = initData.dry_run;
  if (initData.port) q('port').value = initData.port;
  updateLiveStatus(initData.live_status);
  updateBpTuning(initData.brakepoint_tuning);
  updateBpConfig(initData.brakepoint_config);
  updateElevatorSendPwm(initData.elevator_sendpwm);
  poll();
}
init();
</script>
</body>
</html>
"""


class WebGuiState:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.link = SerialLink(self.log)
        self.logs: List[tuple[int, str]] = []
        self.next_log_id = 0
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.mode = "unknown"
        self.fault = "unknown"
        self.live_status = blank_status_snapshot()
        self.live_status_revision = 0
        self.live_status_received_at = 0.0
        self.brakepoint_tuning: Dict[int, Dict[str, str]] = {}
        self.brakepoint_tuning_revision = 0
        self.brakepoint_config: Dict[int, Dict[str, object]] = {
            axis: {
                "slots": {
                    slot: {"MODE": "OFF", "ANGLE": "NA"}
                    for slot in range(1, 5)
                },
            }
            for axis in (1, 2, 3)
        }
        self.brakepoint_config_revision = 0
        self.elevator_sendpwm = {
            "SENDPWM_BTS4": "1.00",
            "SENDPWM_BTS5": "1.00",
        }
        self.elevator_sendpwm_revision = 0
        self.port = args.port or ""
        self.baud = int(args.baud)
        self.dry_run = bool(args.dry_run)

    def log(self, text: str) -> None:
        with self.lock:
            self.logs.append((self.next_log_id, text))
            self.next_log_id += 1
            if len(self.logs) > 2000:
                self.logs = self.logs[-1200:]

    def connect(self, port: str, baud: int, dry_run: bool) -> None:
        self.link.connect(port, baud, dry_run)
        with self.lock:
            self.port = port
            self.baud = baud
            self.dry_run = dry_run

    def disconnect(self) -> None:
        if self.link.is_open():
            self.link.send("failsafe linkloss")
        self.link.close()
        self.log("LOCAL: disconnected")

    def send(self, command: str) -> None:
        self.link.send(command)

    def send_many(self, commands: Iterable[str]) -> None:
        for command in commands:
            self.send(command)

    def lines_since(self, since: int) -> List[tuple[int, str]]:
        with self.lock:
            return [item for item in self.logs if item[0] >= since]

    def next_id(self) -> int:
        with self.lock:
            return self.next_log_id

    def live_status_snapshot(self) -> Dict[str, object]:
        with self.lock:
            return self._live_status_snapshot_locked()

    def _live_status_snapshot_locked(self) -> Dict[str, object]:
        axes = self.live_status.get("axes", [])
        if isinstance(axes, list):
            axes_copy = [axis.copy() if isinstance(axis, dict) else {} for axis in axes]
        else:
            axes_copy = blank_status_axes()
        servo = self.live_status.get("servo", {})
        servo_copy = servo.copy() if isinstance(servo, dict) else {}
        drill_leds = self.live_status.get("drill_leds", [])
        drill_leds_copy = list(drill_leds) if isinstance(drill_leds, list) else ["-"] * 4
        return {
            "mode": self.live_status.get("mode", "-"),
            "stopmode": self.live_status.get("stopmode", "-"),
            "axes": axes_copy,
            "servo": servo_copy,
            "drill_leds": drill_leds_copy,
            "revision": self.live_status_revision,
            "received_at": self.live_status_received_at,
        }

    def _brakepoint_tuning_snapshot_locked(self) -> Dict[str, object]:
        return {
            "axes": {axis: values.copy() for axis, values in self.brakepoint_tuning.items()},
            "revision": self.brakepoint_tuning_revision,
        }

    def _brakepoint_config_snapshot_locked(self) -> Dict[str, object]:
        axes: Dict[int, Dict[str, object]] = {}
        for axis, config in self.brakepoint_config.items():
            slots = config.get("slots", {})
            axes[axis] = {
                "slots": {
                    slot: values.copy()
                    for slot, values in slots.items()
                } if isinstance(slots, dict) else {},
            }
        return {
            "axes": axes,
            "revision": self.brakepoint_config_revision,
        }

    def _elevator_sendpwm_snapshot_locked(self) -> Dict[str, object]:
        return {
            "values": self.elevator_sendpwm.copy(),
            "revision": self.elevator_sendpwm_revision,
        }

    def init_snapshot(self) -> Dict[str, object]:
        with self.lock:
            return {
                "port": self.port,
                "baud": self.baud,
                "dry_run": self.dry_run,
                "live_status": self._live_status_snapshot_locked(),
                "brakepoint_tuning": self._brakepoint_tuning_snapshot_locked(),
                "brakepoint_config": self._brakepoint_config_snapshot_locked(),
                "elevator_sendpwm": self._elevator_sendpwm_snapshot_locked(),
            }

    def poll_snapshot(self, since: int) -> Dict[str, object]:
        connected = self.link.is_open()
        with self.lock:
            return {
                "lines": [item for item in self.logs if item[0] >= since],
                "next": self.next_log_id,
                "connected": connected,
                "mode": self.mode,
                "fault": self.fault,
                "live_status": self._live_status_snapshot_locked(),
                "brakepoint_tuning": self._brakepoint_tuning_snapshot_locked(),
                "brakepoint_config": self._brakepoint_config_snapshot_locked(),
                "elevator_sendpwm": self._elevator_sendpwm_snapshot_locked(),
            }

    def update_status_from_line(self, line: str) -> bool:
        status = parse_status_line(line)
        if status is not None:
            with self.lock:
                previous_servo = self.live_status.get("servo", {})
                previous_leds = self.live_status.get("drill_leds", [])
                if isinstance(previous_servo, dict):
                    status["servo"] = previous_servo.copy()
                if isinstance(previous_leds, list):
                    status["drill_leds"] = list(previous_leds)
                self.live_status = status
                self.live_status_revision += 1
                self.live_status_received_at = time.time()
                self.mode = str(status.get("mode", "-"))
            return True
        servo_status = parse_servo_status_line(line)
        if servo_status is not None:
            with self.lock:
                previous_servo = self.live_status.get("servo", {})
                merged_servo = (
                    previous_servo.copy()
                    if isinstance(previous_servo, dict)
                    else {}
                )
                merged_servo.update(servo_status)
                self.live_status["servo"] = merged_servo
                self.live_status_revision += 1
                self.live_status_received_at = time.time()
            return True
        servo_extended = parse_servo_extended_line(line)
        if servo_extended is not None:
            with self.lock:
                previous_servo = self.live_status.get("servo", {})
                merged_servo = (
                    previous_servo.copy()
                    if isinstance(previous_servo, dict)
                    else {}
                )
                merged_servo.update(servo_extended)
                self.live_status["servo"] = merged_servo
                self.live_status_revision += 1
                self.live_status_received_at = time.time()
            return True
        drill_leds = parse_drill_led_status_line(line)
        if drill_leds is not None:
            with self.lock:
                self.live_status["drill_leds"] = list(drill_leds)
                self.live_status_revision += 1
                self.live_status_received_at = time.time()
            return True
        bp_tuning = parse_brakepoint_tuning_line(line)
        if bp_tuning is not None:
            axis, values = bp_tuning
            with self.lock:
                self.brakepoint_tuning[axis] = values.copy()
                self.brakepoint_tuning_revision += 1
        bp_slot = parse_brakepoint_slot_line(line)
        if bp_slot is not None:
            axis, slot, values = bp_slot
            with self.lock:
                slots = self.brakepoint_config[axis]["slots"]
                assert isinstance(slots, dict)
                slots[slot] = values.copy()
                self.brakepoint_config_revision += 1
        elevator_params = parse_elevator_params_line(line)
        if elevator_params is not None:
            with self.lock:
                self.elevator_sendpwm = elevator_params.copy()
                self.elevator_sendpwm_revision += 1
        if line.startswith("OK MODE "):
            with self.lock:
                self.mode = line.removeprefix("OK MODE ").strip()
        elif line.startswith("OK FAULT "):
            with self.lock:
                self.fault = line.removeprefix("OK FAULT ").strip()
        return False

    def start(self) -> threading.Thread:
        thread = threading.Thread(target=self._poll_loop, daemon=True)
        thread.start()
        return thread

    def _poll_loop(self) -> None:
        next_heartbeat = time.monotonic()
        while not self.stop_event.is_set():
            now = time.monotonic()
            if self.link.is_open() and now >= next_heartbeat:
                self.link.send("hb", log_tx=False)
                next_heartbeat = now + HEARTBEAT_INTERVAL_SECONDS
            for line in self.link.read_lines():
                if not self.update_status_from_line(line):
                    self.log(line if line.startswith("LOCAL:") else f"F401: {line}")
            time.sleep(0.03)

    def shutdown(self) -> None:
        self.stop_event.set()
        if self.link.is_open():
            try:
                self.link.send("failsafe linkloss")
            finally:
                self.link.close()


def make_web_handler(state: WebGuiState) -> type[BaseHTTPRequestHandler]:
    class WebHandler(BaseHTTPRequestHandler):
        def log_message(self, _format: str, *_args: object) -> None:
            return

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            if parsed.path == "/":
                self._send_bytes(WEB_HTML.encode("utf-8"), "text/html; charset=utf-8")
            elif parsed.path == "/api/init":
                self._send_json(state.init_snapshot())
            elif parsed.path == "/api/ports":
                self._send_json({"ports": serial_ports()})
            elif parsed.path == "/api/logs":
                since_text = parse_qs(parsed.query).get("since", ["0"])[0]
                try:
                    since = int(since_text)
                except ValueError:
                    since = 0
                self._send_json(state.poll_snapshot(since))
            else:
                self.send_error(404, "not found")

        def do_POST(self) -> None:
            parsed = urlparse(self.path)
            try:
                data = self._read_json()
                if parsed.path == "/api/send":
                    state.send(str(data.get("cmd", "")))
                    self._send_json({"ok": True})
                elif parsed.path == "/api/send_many":
                    commands = data.get("cmds", [])
                    if not isinstance(commands, list):
                        raise ValueError("cmds must be a list")
                    state.send_many(str(command) for command in commands)
                    self._send_json({"ok": True})
                elif parsed.path == "/api/connect":
                    state.connect(
                        str(data.get("port", "")),
                        int(data.get("baud", DEFAULT_BAUD)),
                        bool(data.get("dry_run", False)),
                    )
                    self._send_json({"ok": True})
                elif parsed.path == "/api/disconnect":
                    state.disconnect()
                    self._send_json({"ok": True})
                else:
                    self.send_error(404, "not found")
            except Exception as exc:
                self.send_response(400)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write(str(exc).encode("utf-8", errors="replace"))

        def _read_json(self) -> dict:
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length) if length else b"{}"
            if not raw:
                return {}
            decoded = raw.decode("utf-8", errors="replace")
            data = json.loads(decoded)
            if not isinstance(data, dict):
                raise ValueError("request body must be a JSON object")
            return data

        def _send_json(self, data: object) -> None:
            self._send_bytes(json.dumps(data).encode("utf-8"), "application/json; charset=utf-8")

        def _send_bytes(self, body: bytes, content_type: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

    return WebHandler


def run_web_gui(args: argparse.Namespace) -> int:
    state = WebGuiState(args)
    if args.auto_connect or args.dry_run:
        try:
            state.connect(args.port or "", int(args.baud), bool(args.dry_run))
        except Exception as exc:
            state.log(f"LOCAL: auto-connect failed: {exc}")
    state.start()

    server = ThreadingHTTPServer((args.web_host, args.web_port), make_web_handler(state))
    url = f"http://{args.web_host}:{args.web_port}/"
    print(f"Web GUI ready: {url}")
    print("Close with Ctrl+C. Shutdown sends failsafe linkloss.")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping web GUI...")
    finally:
        state.shutdown()
        server.server_close()
    return 0


def run_self_test() -> None:
    class FakeSerial:
        def __init__(self) -> None:
            self.rx = bytearray()
            self.tx = bytearray()
            self.closed = False

        @property
        def in_waiting(self) -> int:
            return len(self.rx)

        def feed(self, data: bytes) -> None:
            self.rx.extend(data)

        def read(self, size: int) -> bytes:
            data = bytes(self.rx[:size])
            del self.rx[:size]
            return data

        def write(self, data: bytes) -> int:
            self.tx.extend(data)
            return len(data)

        def close(self) -> None:
            self.closed = True

    assert clamp_int(300, 0, 255) == 255
    assert clamp_int(-5, 0, 255) == 0
    assert clamp_int(120, 0, 255) == 120
    assert scale_gamepad_axis(0.04, False, 50) == 0
    assert scale_gamepad_axis(0.75, False, 50) == 750
    assert scale_gamepad_axis(0.75, True, 50) == -750
    assert scale_gamepad_axis(2.0, False, 50) == 1000
    assert len(GAMEPAD_AXIS_DEFAULTS) == 4
    assert all(target != 7 for _name, target, _axis, _invert in GAMEPAD_AXIS_DEFAULTS)
    try:
        servo_tube1_reference_bounds(45, False)
        raise AssertionError("five 45-degree states must exceed 165-degree travel")
    except ValueError:
        pass
    try:
        servo_tube1_reference_bounds(45, True)
        raise AssertionError("five inverted 45-degree states must exceed travel")
    except ValueError:
        pass
    assert STOP_MODES == ("coast", "brake", "hold", "hybrid")
    assert AXIS_NAMES[3] == "J4 Pitch"
    status_axes = [
        "J1_PWM:120 J1_DIR:forward J1_EN:1 J1_ANGLE:12.34 J1_VEL_DPS:-5.67"
    ]
    status_axes.extend(
        f"J{axis}_PWM:0 J{axis}_DIR:stop J{axis}_EN:0 J{axis}_ANGLE:NA J{axis}_VEL_DPS:NA"
        for axis in range(2, 7)
    )
    servo_status_line = (
        "SERVO INIT:1 STATE:2 TARGET:5 MOVING:1 ANGLE:45 WIGGLE:5 "
        "INVERT:0 SPEED:30 PWM:1 PULSE:1320 TARGETPULSE:2073 SAFETY:READY"
    )
    servo_reference_line = (
        "SERVOREFX VALID:1 CURRENTSTATE:2 TARGETSTATE:5 CURRENTPULSE:1320 "
        "REFERENCEPULSE:970 TARGETPULSE:2020 ERROR:NONE"
    )
    servo_manual_line = (
        "SERVOMANUAL VALUE:500 ENABLED:1 ACTIVE:1 NEUTRAL:1 CONFIGSPEED:5 "
        "ACTIVESPEED:5 ANGLESTEP:45 INVERT:0 PWM:1"
    )
    servo_angles_line = (
        "SERVOANGLES VALID:1 CURRENT:45.0 TARGET:90.0 REFERENCE:0.0"
    )
    servo_calibration_line = (
        "SERVOCAL MASK:27 P1:820 P2:1150 P3:NA P4:1850 P5:2180"
    )
    drill_led_status_line = "DRILLLEDS LED1:1 LED2:0 LED3:1 LED4:0"
    full_status = "MODE:arm STOPMODE:coast " + " ".join(status_axes)
    parsed = parse_status_line(full_status)
    assert parsed is not None
    assert parsed["mode"] == "arm"
    assert parsed["stopmode"] == "coast"
    axes = parsed["axes"]
    assert isinstance(axes, list)
    assert axes[0]["PWM"] == "120"
    assert axes[0]["VEL_DPS"] == "-5.67"
    assert axes[5]["ANGLE"] == "NA"
    parsed_servo = parse_servo_status_line(servo_status_line)
    assert parsed_servo is not None
    assert parsed_servo["SERVO_STATE"] == "2"
    assert parsed_servo["SERVO_TARGET"] == "5"
    assert parsed_servo["SERVOSPEED"] == "30"
    assert parsed_servo["SERVO_PWM"] == "1"
    assert parsed_servo["SERVO_PULSE"] == "1320"
    assert parsed_servo["SERVO_TARGETPULSE"] == "2073"
    assert parsed_servo["SERVO_SAFETY"] == "READY"
    parsed_reference = parse_servo_extended_line(servo_reference_line)
    assert parsed_reference is not None
    assert parsed_reference["SERVO_REFERENCE_VALID"] == "1"
    assert parsed_reference["SERVO_REFERENCE_PULSE"] == "970"
    parsed_manual = parse_servo_extended_line(servo_manual_line)
    assert parsed_manual is not None
    assert parsed_manual["SERVO_MANUAL_VALUE"] == "500"
    assert parsed_manual["SERVO_MANUAL_ENABLED"] == "1"
    assert parsed_manual["SERVO_CONFIGURED_SPEED"] == "5"
    assert parse_servo_extended_line(
        servo_manual_line.replace("CONFIGSPEED:5", "CONFIGSPEED:0")
    ) is None
    parsed_angles = parse_servo_extended_line(servo_angles_line)
    assert parsed_angles is not None
    assert parsed_angles["SERVO_CURRENT_ANGLE"] == "45.0"
    parsed_calibration = parse_servo_extended_line(servo_calibration_line)
    assert parsed_calibration is not None
    assert parsed_calibration["SERVO_CAL_MASK"] == "27"
    assert parsed_calibration["SERVO_CAL_P3"] == "NA"
    assert parsed_calibration["SERVO_CAL_P5"] == "2180"
    assert parse_servo_extended_line(
        servo_calibration_line.replace("P3:NA", "P3:1700")
    ) is None
    safe_servo_line = (
        "SERVO INIT:0 STATE:NA TARGET:NA MOVING:0 ANGLE:45 WIGGLE:5 "
        "INVERT:0 SPEED:30 PWM:0 PULSE:1500 TARGETPULSE:1500 SAFETY:LINK_LOSS"
    )
    assert parse_servo_status_line(safe_servo_line) is not None
    assert parse_servo_status_line(
        servo_status_line.replace("SPEED:30", "SPEED:0")
    ) is None
    assert parse_servo_status_line(
        servo_status_line.replace("TARGET:5", "TARGET:9")
    ) is None
    assert parse_drill_led_status_line(drill_led_status_line) == ["1", "0", "1", "0"]
    armtest_status = parse_status_line(
        "MODE:armtest STOPMODE:coast " + " ".join(status_axes)
    )
    assert armtest_status is not None
    assert armtest_status["mode"] == "armtest"
    assert parse_status_line("MODE:arm STOPMODE:coast J1_PWM:120") is None
    corrupt_status = (
        "MODE:arm STOPMODE:coast "
        + " ".join(status_axes).replace("J3_PWM:0", "J3_PWM:999")
    )
    assert parse_status_line(corrupt_status) is None

    bp_params_line = (
        "J2 INVERT=0 MAXPWM=255 "
        "BPREACT_MS=40 BPDECEL_DPS2=350 BPLEAD_DEG=0.80 "
        "BPCLEAR_DEG=1.00 BPRECPWM=90 BPRECTIME_MS=3500 KEYFWD=-1"
    )
    parsed_bp = parse_brakepoint_tuning_line(bp_params_line)
    assert parsed_bp is not None
    bp_axis, bp_values = parsed_bp
    assert bp_axis == 2
    assert bp_values["BPDECEL_DPS2"] == "350"
    assert bp_values["BPCLEAR_DEG"] == "1.00"
    assert parse_brakepoint_tuning_line(bp_params_line.replace("J2", "J4", 1)) is None
    assert parse_brakepoint_tuning_line(bp_params_line.replace("BPRECPWM=90", "BPRECPWM=999")) is None
    bp_slot_line = "BP J2 SLOT=3 MODE=BLOCK_INCREASE ANGLE=90.00"
    parsed_slot = parse_brakepoint_slot_line(bp_slot_line)
    assert parsed_slot == (
        2,
        3,
        {"MODE": "BLOCK_INCREASE", "ANGLE": "90.00"},
    )
    assert parse_brakepoint_slot_line(
        "BP J2 SLOT=5 MODE=WALL ANGLE=90.00"
    ) is None
    elevator_params_line = (
        "ELV INVERT=0 MAXPWM=255 DEFAULT=100 "
        "SENDPWM_BTS4=1.00 SENDPWM_BTS5=0.75"
    )
    parsed_elevator = parse_elevator_params_line(elevator_params_line)
    assert parsed_elevator == {
        "SENDPWM_BTS4": "1.00",
        "SENDPWM_BTS5": "0.75",
    }
    assert parse_elevator_params_line(
        elevator_params_line.replace("SENDPWM_BTS5=0.75", "SENDPWM_BTS5=1.01")
    ) is None
    state_args = argparse.Namespace(port=None, baud=DEFAULT_BAUD, dry_run=True)
    web_state = WebGuiState(state_args)
    web_state.connect("", DEFAULT_BAUD, True)
    assert web_state.update_status_from_line(full_status)
    assert web_state.update_status_from_line(servo_status_line)
    assert web_state.update_status_from_line(servo_reference_line)
    assert web_state.update_status_from_line(servo_manual_line)
    assert web_state.update_status_from_line(servo_angles_line)
    assert web_state.update_status_from_line(servo_calibration_line)
    assert web_state.update_status_from_line(drill_led_status_line)
    live_snapshot = web_state.live_status_snapshot()
    assert live_snapshot["revision"] == 7
    assert live_snapshot["received_at"] > 0
    assert live_snapshot["servo"]["SERVO_STATE"] == "2"
    assert live_snapshot["servo"]["SERVO_TARGET"] == "5"
    assert live_snapshot["servo"]["SERVO_SAFETY"] == "READY"
    assert live_snapshot["servo"]["SERVO_REFERENCE_VALID"] == "1"
    assert live_snapshot["servo"]["SERVO_CONFIGURED_SPEED"] == "5"
    assert live_snapshot["servo"]["SERVO_CURRENT_ANGLE"] == "45.0"
    assert live_snapshot["servo"]["SERVO_CAL_P4"] == "1850"
    assert live_snapshot["servo"]["SERVO_CAL_P5"] == "2180"
    assert live_snapshot["drill_leds"] == ["1", "0", "1", "0"]
    poll_snapshot = web_state.poll_snapshot(0)
    assert poll_snapshot["connected"]
    assert poll_snapshot["next"] == len(poll_snapshot["lines"])
    assert not web_state.update_status_from_line(bp_params_line)
    bp_snapshot = web_state.poll_snapshot(0)["brakepoint_tuning"]
    assert isinstance(bp_snapshot, dict)
    assert bp_snapshot["revision"] == 1
    assert bp_snapshot["axes"][2]["BPRECPWM"] == "90"
    assert not web_state.update_status_from_line(bp_slot_line)
    bp_config_snapshot = web_state.poll_snapshot(0)["brakepoint_config"]
    assert isinstance(bp_config_snapshot, dict)
    assert bp_config_snapshot["revision"] == 1
    assert bp_config_snapshot["axes"][2]["slots"][3]["MODE"] == "BLOCK_INCREASE"
    assert not web_state.update_status_from_line(elevator_params_line)
    elevator_snapshot = web_state.poll_snapshot(0)["elevator_sendpwm"]
    assert isinstance(elevator_snapshot, dict)
    assert elevator_snapshot["revision"] == 1
    assert elevator_snapshot["values"]["SENDPWM_BTS5"] == "0.75"
    web_state.link.close()
    assert "stopall" in WEB_HTML
    assert "Active hold preset" in WEB_HTML
    assert "liveStatusRows" in WEB_HTML
    assert "mode armtest confirm" in WEB_HTML
    assert "BLOCK_INCREASE" in WEB_HTML
    for command in ("bpreact", "bpdecel", "bplead", "bpclear", "bprecpwm", "bprectime"):
        assert command in WEB_HTML
    assert "set bts${bts} sendpwm ${normalized}" in WEB_HTML
    assert "Params / load" in WEB_HTML
    assert "set servostate" in WEB_HTML
    assert "set servopulse" not in WEB_HTML
    assert "set servospeed" in WEB_HTML
    assert "SERVO_CURRENT_ANGLE" in WEB_HTML
    assert "servoStateLabel" in WEB_HTML
    for pulse in (1000, 1500, 2000):
        assert f"servo pulse {pulse}" in WEB_HTML
    assert "servo pulse off" in WEB_HTML
    assert "servo diag" in WEB_HTML
    assert "servo backward '+v('servoStep')" in WEB_HTML
    assert "servo forward '+v('servoStep')" in WEB_HTML
    assert 'id="servoStep"' in WEB_HTML
    assert "onmousedown" not in WEB_HTML
    assert "startServoManualButton" not in WEB_HTML
    assert "servo jog" not in WEB_HTML
    assert "Fallback angle" not in WEB_HTML
    assert "Save current as Tube 4" in WEB_HTML
    assert "Save current as Final lid" in WEB_HTML
    assert "servo 5" in WEB_HTML
    assert "Servo manual" not in WEB_HTML
    assert "Calibration rejected at" in WEB_HTML
    assert "Enable Manual Control" not in WEB_HTML
    assert "Apply once" in WEB_HTML
    assert "joy disconnect" in WEB_HTML
    assert "servowiggle" in WEB_HTML
    assert "DrillLed 4 1" in WEB_HTML

    fake = FakeSerial()
    serial_log: List[str] = []
    link = SerialLink(serial_log.append)
    link.serial = fake
    fake.feed(b"MODE:arm STOPMODE:co")
    assert link.read_lines() == []
    fake.feed(b"ast J1_PWM:12\r\nOK FIRST\nOK SECOND\r\n")
    assert link.read_lines(max_lines=2) == ["MODE:arm STOPMODE:coast J1_PWM:12", "OK FIRST"]
    assert link.read_lines() == ["OK SECOND"]
    link.send("get mode", log_tx=False)
    assert fake.tx == b"get mode\n"

    fake.feed(b"x" * (MAX_SERIAL_LINE_BYTES + 1))
    assert link.read_lines() == []
    assert link.read_lines() == [
        f"LOCAL: discarded serial line longer than {MAX_SERIAL_LINE_BYTES} bytes"
    ]
    fake.feed(b"discarded suffix\nOK RECOVERED\n")
    assert link.read_lines() == ["OK RECOVERED"]
    link.close()
    assert fake.closed
    print("self-test ok")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Test GUI for the F401 manipulation/drill firmware.")
    parser.add_argument("--port", help="FTDI serial port, for example COM7.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--dry-run", action="store_true", help="Open GUI without serial hardware; only log TX lines.")
    parser.add_argument("--auto-connect", action="store_true", help="Connect automatically on startup.")
    parser.add_argument("--web", action="store_true", help="Force browser-based GUI instead of Tkinter.")
    parser.add_argument("--web-host", default=DEFAULT_WEB_HOST)
    parser.add_argument("--web-port", type=int, default=DEFAULT_WEB_PORT)
    parser.add_argument("--no-browser", action="store_true", help="Do not automatically open the browser for --web.")
    parser.add_argument("--list-ports", action="store_true", help="Print serial ports and exit.")
    parser.add_argument("--self-test", action="store_true", help="Run non-GUI checks and exit.")
    parser.add_argument("--gamepad-helper", choices=("list", "stream"), help=argparse.SUPPRESS)
    parser.add_argument("--gamepad-index", type=int, default=0, help=argparse.SUPPRESS)
    parser.add_argument("--gamepad-helper-poll-ms", type=int, default=20, help=argparse.SUPPRESS)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.gamepad_helper:
        return run_gamepad_helper(args)
    if args.self_test:
        run_self_test()
        return 0
    if args.list_ports:
        for port in serial_ports():
            print(port)
        return 0

    if args.web or tk is None:
        if tk is None and not args.web:
            print("tkinter is not available in this Python. Starting browser-based GUI instead.")
        return run_web_gui(args)

    root = tk.Tk()
    F401TestGui(root, args)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
