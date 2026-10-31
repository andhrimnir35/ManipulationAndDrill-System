# Requires Python 3. Run with the interpreter configured in .vscode/settings.json.
"""
Direct PC -> FTDI -> STM32F401 joystick bridge for bench testing.

Dependencies:
  python -m pip install pygame pyserial

Typical use:
  python tools/joystick_direct_f401.py --port COM7 --arm

While it is running, type normal F401 commands in the same terminal. The script
also prints all F401 replies, so it replaces PlatformIO Serial Monitor for this
direct bench setup.

Event id discovery:
  python tools/joystick_direct_f401.py --event-test

Serial monitor without joystick:
  python tools/joystick_direct_f401.py --port COM7 --monitor-only
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import queue
import sys
import threading
import time
from typing import Deque, Dict, Iterable, List, Optional, Tuple

from python_deps import ensure_pyserial_available


ensure_pyserial_available()


HAT_BUTTON_BASE = 1000
HAT_UP_OFFSET = 0
HAT_DOWN_OFFSET = 1
HAT_LEFT_OFFSET = 2
HAT_RIGHT_OFFSET = 3
MAX_SERIAL_LINE_BYTES = 4096
SERIAL_READ_CHUNK_BYTES = 4096


_print_lock = threading.Lock()


def console_print(text: str = "") -> None:
    with _print_lock:
        print(text, flush=True)


@dataclass(frozen=True)
class AxisBinding:
    joint: int
    axis: int
    invert: bool
    name: str


@dataclass
class BridgeState:
    joystick_enabled: bool = True
    echo_manual_tx: bool = True
    echo_joystick_tx: bool = False
    force_joystick_refresh: bool = True


class CommandSink:
    def __init__(self, port: Optional[str], baud: int, dry_run: bool) -> None:
        self.dry_run = dry_run
        self.serial = None
        self._write_lock = threading.Lock()
        self._read_lock = threading.Lock()
        self._rx_buffer = bytearray()
        self._rx_lines: Deque[str] = deque()
        self._discarding_oversize_line = False
        if dry_run:
            return
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError("pyserial missing. Install with: python -m pip install pyserial") from exc
        if not port:
            raise RuntimeError("Serial port is required unless --dry-run or --event-test is used.")
        self.serial = serial.Serial(port, baudrate=baud, timeout=0.02, write_timeout=0.2)
        time.sleep(0.2)

    def send(self, line: str, *, echo: bool = False, prefix: str = "TX") -> None:
        line = line.strip()
        if not line:
            return
        if self.dry_run:
            console_print(f"{prefix}: {line}")
            return
        if echo:
            console_print(f"{prefix}: {line}")
        with self._write_lock:
            if self.serial is None:
                return
            self.serial.write((line + "\n").encode("ascii", errors="ignore"))

    def readline(self) -> Optional[str]:
        if self.dry_run or self.serial is None:
            return None
        with self._read_lock:
            if self.serial is None:
                return None
            if self._rx_lines:
                return self._rx_lines.popleft()
            waiting = int(getattr(self.serial, "in_waiting", 0))
            raw = self.serial.read(min(waiting, SERIAL_READ_CHUNK_BYTES)) if waiting > 0 else self.serial.read(1)
            if not raw:
                return None
            self._ingest_bytes(raw)
            return self._rx_lines.popleft() if self._rx_lines else None

    def _ingest_bytes(self, raw: bytes) -> None:
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

    def close(self) -> None:
        with self._write_lock:
            with self._read_lock:
                if self.serial is not None:
                    try:
                        self.serial.close()
                    finally:
                        self.serial = None
                self._rx_buffer.clear()
                self._rx_lines.clear()
                self._discarding_oversize_line = False


def hat_button_id(hat_index: int, direction: str) -> int:
    offsets = {
        "up": HAT_UP_OFFSET,
        "down": HAT_DOWN_OFFSET,
        "left": HAT_LEFT_OFFSET,
        "right": HAT_RIGHT_OFFSET,
    }
    return HAT_BUTTON_BASE + hat_index * 4 + offsets[direction]


def scale_axis(raw_value: float, invert: bool, output_dead: int) -> int:
    value = -raw_value if invert else raw_value
    if value > 1.0:
        value = 1.0
    elif value < -1.0:
        value = -1.0
    scaled = int(round(value * 1000.0))
    if abs(scaled) <= output_dead:
        return 0
    return scaled


def default_axis_bindings(axis_count: int) -> List[AxisBinding]:
    # Common Xbox-style pygame mapping:
    # 0=LX, 1=LY, 3=RX, 4=RY. Some generic controllers expose 2=RX, 3=RY.
    if axis_count >= 5:
        return [
            AxisBinding(1, 0, False, "left_x -> base"),
            AxisBinding(2, 1, True, "left_y -> shoulder"),
            AxisBinding(3, 4, True, "right_y -> elbow"),
            AxisBinding(5, 3, False, "right_x -> twist"),
        ]
    return [
        AxisBinding(1, 0, False, "left_x -> base"),
        AxisBinding(2, 1, True, "left_y -> shoulder"),
        AxisBinding(3, 3, True, "right_y -> elbow"),
        AxisBinding(5, 2, False, "right_x -> twist"),
    ]


def parse_axis_map(spec: str) -> List[AxisBinding]:
    # Format: "1:0,2:-1,3:-4,5:3"
    # Negative axis spec means invert that joystick axis, not a negative axis id.
    bindings: List[AxisBinding] = []
    for raw_item in spec.split(","):
        item = raw_item.strip()
        if not item:
            continue
        if ":" not in item:
            raise ValueError(f"Bad axis map item: {item}")
        joint_text, axis_text = item.split(":", 1)
        joint = int(joint_text)
        if joint < 1 or joint > 6:
            raise ValueError(f"Joint out of range in axis map: {joint}")
        invert = axis_text.startswith("-")
        if axis_text.startswith("-") or axis_text.startswith("+"):
            axis_text = axis_text[1:]
        axis = int(axis_text)
        if axis < 0:
            raise ValueError(f"Axis id out of range in axis map: {axis}")
        bindings.append(AxisBinding(joint, axis, invert, f"custom axis {axis} -> J{joint}"))
    return bindings


def start_serial_reader(sink: CommandSink, stop_event: threading.Event) -> threading.Thread:
    def run() -> None:
        while not stop_event.is_set():
            line = sink.readline()
            if line:
                console_print(f"F401: {line}")
            else:
                time.sleep(0.005)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread


def start_console_reader(stop_event: threading.Event) -> Tuple[queue.Queue[str], threading.Thread]:
    lines: queue.Queue[str] = queue.Queue()

    def run() -> None:
        while not stop_event.is_set():
            line = sys.stdin.readline()
            if line == "":
                break
            line = line.strip()
            if line:
                lines.put(line)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return lines, thread


def list_ports() -> None:
    try:
        from serial.tools import list_ports  # type: ignore
    except ImportError:
        console_print("pyserial missing. Install with: python -m pip install pyserial")
        return
    for port in list_ports.comports():
        console_print(f"{port.device}: {port.description}")


def print_mapping(bindings: Iterable[AxisBinding], button_count: int, hat_count: int) -> None:
    console_print("Axis mapping:")
    for binding in bindings:
        sign = "-" if binding.invert else "+"
        console_print(f"  joystick axis {sign}{binding.axis} -> axis {binding.joint} ({binding.name})")
    console_print("Button forwarding:")
    console_print(f"  physical buttons: joy button <0..{max(button_count - 1, 0)}> <0|1>")
    for hat in range(hat_count):
        console_print(f"  hat {hat} up    -> joy button {hat_button_id(hat, 'up')}")
        console_print(f"  hat {hat} down  -> joy button {hat_button_id(hat, 'down')}")
        console_print(f"  hat {hat} left  -> joy button {hat_button_id(hat, 'left')}")
        console_print(f"  hat {hat} right -> joy button {hat_button_id(hat, 'right')}")


def print_console_help() -> None:
    console_print("Interactive serial monitor commands:")
    console_print("  Type any F401 command and press Enter, for example: params")
    console_print("  /pause       stop arm motion and pause joystick TX")
    console_print("  /resume      resume joystick TX")
    console_print("  /stop        send stopall")
    console_print("  /safe        send stopall, then mode safe")
    console_print("  /arm         send mode arm confirm and resume joystick")
    console_print("  /drill       send mode drill confirm and pause joystick")
    console_print("  /echo on|off show/hide manual TX lines")
    console_print("  /joyecho on|off show/hide joystick TX lines")
    console_print("  /status      show local bridge state")
    console_print("  /quit        send stopall and exit")


def handle_console_line(
    line: str,
    sink: CommandSink,
    state: BridgeState,
    stop_event: threading.Event,
) -> None:
    stripped = line.strip()
    if not stripped:
        return

    lower = stripped.lower()
    if not lower.startswith("/"):
        sink.send(stripped, echo=state.echo_manual_tx, prefix="TX")
        if lower in {"quit", "exit"}:
            stop_event.set()
        return

    parts = lower.split()
    command = parts[0]
    argument = parts[1] if len(parts) >= 2 else ""

    if command in {"/help", "/?"}:
        print_console_help()
    elif command == "/pause":
        state.joystick_enabled = False
        sink.send("stop", echo=state.echo_manual_tx, prefix="TX")
        console_print("LOCAL: joystick paused")
    elif command == "/resume":
        state.joystick_enabled = True
        state.force_joystick_refresh = True
        console_print("LOCAL: joystick resumed")
    elif command == "/stop":
        sink.send("stopall", echo=state.echo_manual_tx, prefix="TX")
    elif command == "/safe":
        state.joystick_enabled = False
        sink.send("stopall", echo=state.echo_manual_tx, prefix="TX")
        sink.send("mode safe", echo=state.echo_manual_tx, prefix="TX")
    elif command == "/arm":
        state.joystick_enabled = True
        state.force_joystick_refresh = True
        sink.send("mode arm confirm", echo=state.echo_manual_tx, prefix="TX")
    elif command == "/drill":
        state.joystick_enabled = False
        sink.send("mode drill confirm", echo=state.echo_manual_tx, prefix="TX")
    elif command == "/echo" and argument in {"on", "off"}:
        state.echo_manual_tx = argument == "on"
        console_print(f"LOCAL: manual TX echo {'on' if state.echo_manual_tx else 'off'}")
    elif command == "/joyecho" and argument in {"on", "off"}:
        state.echo_joystick_tx = argument == "on"
        console_print(f"LOCAL: joystick TX echo {'on' if state.echo_joystick_tx else 'off'}")
    elif command == "/status":
        console_print(
            "LOCAL: "
            f"joystick={'on' if state.joystick_enabled else 'paused'} "
            f"manual_echo={'on' if state.echo_manual_tx else 'off'} "
            f"joy_echo={'on' if state.echo_joystick_tx else 'off'}"
        )
    elif command in {"/quit", "/exit"}:
        sink.send("stopall", echo=state.echo_manual_tx, prefix="TX")
        stop_event.set()
    else:
        console_print(f"LOCAL: unknown command {stripped}. Type /help.")


def run_self_test() -> None:
    class FakeSerial:
        def __init__(self) -> None:
            self.rx = bytearray()
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

        def close(self) -> None:
            self.closed = True

    bindings = default_axis_bindings(axis_count=6)
    assert scale_axis(0.50, False, 10) == 500
    assert scale_axis(-1.0, True, 10) == 1000
    assert scale_axis(0.004, False, 10) == 0
    assert [b.joint for b in bindings] == [1, 2, 3, 5]
    assert [b.axis for b in bindings] == [0, 1, 4, 3]
    assert hat_button_id(0, "up") == 1000
    assert hat_button_id(0, "down") == 1001
    assert hat_button_id(0, "left") == 1002
    assert hat_button_id(0, "right") == 1003
    custom = parse_axis_map("1:0,2:-1,3:-4,5:3")
    assert custom[1].joint == 2 and custom[1].axis == 1 and custom[1].invert
    state = BridgeState()
    assert state.joystick_enabled and state.echo_manual_tx and not state.echo_joystick_tx

    fake = FakeSerial()
    sink = CommandSink(None, 115200, dry_run=True)
    sink.dry_run = False
    sink.serial = fake
    fake.feed(b"MODE:arm J1_ANGLE:12")
    assert sink.readline() is None
    fake.feed(b".34\r\nOK NEXT\n")
    assert sink.readline() == "MODE:arm J1_ANGLE:12.34"
    assert sink.readline() == "OK NEXT"
    sink.close()
    assert fake.closed
    console_print("SELF_TEST PASS")


def run_monitor_only(args: argparse.Namespace) -> int:
    sink = CommandSink(args.port, args.baud, args.dry_run)
    stop_event = threading.Event()
    state = BridgeState(joystick_enabled=False)
    if not args.dry_run:
        start_serial_reader(sink, stop_event)
    console_queue, _ = start_console_reader(stop_event)
    console_print("Serial monitor ready. Type F401 commands directly, or /help for local commands.")

    try:
        while not stop_event.is_set():
            try:
                line = console_queue.get(timeout=0.05)
            except queue.Empty:
                continue
            handle_console_line(line, sink, state, stop_event)
    except KeyboardInterrupt:
        sink.send("stopall", echo=state.echo_manual_tx, prefix="TX")
    finally:
        stop_event.set()
        sink.close()
    return 0


def run_bridge(args: argparse.Namespace) -> int:
    if args.monitor_only:
        return run_monitor_only(args)

    try:
        import pygame  # type: ignore
    except ImportError:
        console_print("pygame missing. Install with: python -m pip install pygame")
        return 2

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() <= args.joystick:
        console_print("No joystick found. Use --monitor-only for serial terminal without a gamepad.")
        return 2

    joystick = pygame.joystick.Joystick(args.joystick)
    joystick.init()
    axis_count = joystick.get_numaxes()
    button_count = joystick.get_numbuttons()
    hat_count = joystick.get_numhats()

    bindings = parse_axis_map(args.axis_map) if args.axis_map else default_axis_bindings(axis_count)
    for binding in bindings:
        if binding.axis >= axis_count:
            console_print(f"Axis map references missing joystick axis {binding.axis}. Controller has {axis_count} axes.")
            return 2

    console_print(f"Joystick: {joystick.get_name()}")
    console_print(f"Axes={axis_count} Buttons={button_count} Hats={hat_count}")
    print_mapping(bindings, button_count, hat_count)

    if args.event_test:
        sink = CommandSink(None, args.baud, dry_run=True)
    else:
        sink = CommandSink(args.port, args.baud, args.dry_run)

    state = BridgeState()
    stop_event = threading.Event()
    if not args.event_test and not args.dry_run:
        start_serial_reader(sink, stop_event)
    console_queue: Optional[queue.Queue[str]] = None
    if not args.event_test and not args.no_console:
        console_queue, _ = start_console_reader(stop_event)
        console_print("Serial monitor active. Type F401 commands directly, or /help for local commands.")

    last_axis_values: Dict[int, int] = {}
    last_event_axis_values: Dict[int, int] = {}
    last_button_values = [0] * button_count
    last_hat_direction_values: Dict[int, int] = {}

    try:
        if args.arm and not args.event_test:
            sink.send("mode arm confirm", echo=state.echo_manual_tx, prefix="TX")

        while not stop_event.is_set():
            pygame.event.pump()

            if console_queue is not None:
                while True:
                    try:
                        line = console_queue.get_nowait()
                    except queue.Empty:
                        break
                    handle_console_line(line, sink, state, stop_event)

            if args.event_test:
                for axis in range(axis_count):
                    value = scale_axis(joystick.get_axis(axis), False, args.output_dead)
                    previous = last_event_axis_values.get(axis)
                    if previous is None or abs(value - previous) >= args.event_delta:
                        last_event_axis_values[axis] = value
                        console_print(f"AXIS id={axis} value={value}")
            elif state.joystick_enabled:
                if state.force_joystick_refresh:
                    last_axis_values.clear()
                    last_button_values = [-1] * button_count
                    last_hat_direction_values.clear()
                    state.force_joystick_refresh = False
                for binding in bindings:
                    value = scale_axis(joystick.get_axis(binding.axis), binding.invert, args.output_dead)
                    previous = last_axis_values.get(binding.joint)
                    should_send = previous is None or abs(value - previous) >= args.axis_delta
                    should_send = should_send or (previous != 0 and value == 0)
                    if should_send:
                        last_axis_values[binding.joint] = value
                        sink.send(
                            f"axis {binding.joint} {value}",
                            echo=state.echo_joystick_tx,
                            prefix="JOYTX",
                        )

            if state.joystick_enabled or args.event_test:
                for button in range(button_count):
                    button_state = 1 if joystick.get_button(button) else 0
                    if button_state != last_button_values[button]:
                        last_button_values[button] = button_state
                        if args.event_test or args.print_events:
                            console_print(f"BUTTON id={button} state={button_state}")
                        if not args.event_test and state.joystick_enabled:
                            sink.send(
                                f"joy button {button} {button_state}",
                                echo=state.echo_joystick_tx,
                                prefix="JOYTX",
                            )

                for hat in range(hat_count):
                    x_value, y_value = joystick.get_hat(hat)
                    directions = {
                        "up": 1 if y_value > 0 else 0,
                        "down": 1 if y_value < 0 else 0,
                        "left": 1 if x_value < 0 else 0,
                        "right": 1 if x_value > 0 else 0,
                    }
                    for direction, direction_state in directions.items():
                        synthetic_id = hat_button_id(hat, direction)
                        previous = last_hat_direction_values.get(synthetic_id, 0)
                        if direction_state != previous:
                            last_hat_direction_values[synthetic_id] = direction_state
                            if args.event_test or args.print_events:
                                console_print(
                                    f"HAT hat={hat} direction={direction} id={synthetic_id} state={direction_state}"
                                )
                            if not args.event_test and state.joystick_enabled:
                                sink.send(
                                    f"joy button {synthetic_id} {direction_state}",
                                    echo=state.echo_joystick_tx,
                                    prefix="JOYTX",
                                )

            time.sleep(args.interval)
    except KeyboardInterrupt:
        sink.send("stopall", echo=state.echo_manual_tx, prefix="TX")
    finally:
        stop_event.set()
        sink.close()
        pygame.quit()

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Direct gamepad to STM32F401 serial bridge.")
    parser.add_argument("--port", help="FTDI serial port, for example COM7.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--joystick", type=int, default=0, help="pygame joystick index.")
    parser.add_argument("--axis-map", help='Override axis mapping, example: "1:0,2:-1,3:-4,5:3".')
    parser.add_argument("--interval", type=float, default=0.02, help="Main loop interval in seconds.")
    parser.add_argument("--axis-delta", type=int, default=20, help="Minimum axis command delta.")
    parser.add_argument("--event-delta", type=int, default=80, help="Minimum printed delta in --event-test.")
    parser.add_argument("--output-dead", type=int, default=10, help="Tiny PC-side output deadband before F401 deadband.")
    parser.add_argument("--arm", action="store_true", help="Send mode arm confirm on startup.")
    parser.add_argument("--dry-run", action="store_true", help="Print commands instead of opening serial.")
    parser.add_argument("--event-test", action="store_true", help="Print joystick ids and do not use serial.")
    parser.add_argument("--print-events", action="store_true", help="Print button/hat events while bridging.")
    parser.add_argument("--no-console", action="store_true", help="Do not read stdin commands.")
    parser.add_argument("--monitor-only", action="store_true", help="Open serial terminal without joystick.")
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.self_test:
        run_self_test()
        return 0
    if args.list_ports:
        list_ports()
        return 0
    try:
        return run_bridge(args)
    except Exception as exc:
        console_print(f"ERROR: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
