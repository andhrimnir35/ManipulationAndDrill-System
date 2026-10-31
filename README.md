# Manipulation and Drill System

**STM32F401CEU6 BlackPill** firmware and Python test tools for a rover's robotic arm and drilling mechanism. The firmware uses the Arduino STM32 framework and builds with PlatformIO. The system architecture follows a Raspberry Pi/GUI -> H7 -> F401 command chain; this repository contains the F401 firmware and PC test tools that can connect directly over UART.

## Hardware and operating modes

- **6 x BTS7960:** In ARM mode, control J1 base, J2 shoulder, J3 elbow, the differential wrist (J4 pitch / J5 twist), and J6 gripper.
- **DRILL mode:** BTS4/BTS5 drive the elevator motors, and BTS6 drives the drill motor. Switching between ARM and DRILL also requires the correct physical motor connections; software does not change the wiring.
- **3 x AS5600 + TCA9548A:** Angle/velocity feedback, position control, and breakpoint boundaries for J1-J3.
- **Power HD 1235MG servo:** 50 Hz PWM on PA0/TIM5, calibration, and five-state control.
- **4 active-LOW relays:** PB4, PB13, PB14, PB15.
- **SAFE:** Startup mode with motor outputs disabled. **ARMTEST:** A manual test mode requiring explicit confirmation; breakpoints, goto/rotate, and hold are disabled.

UART: **115200 baud**, PA2=TX / PA3=RX, **3.3 V TTL**, common GND. I2C: PB9=SDA / PB8=SCL; sensors use TCA channels 0/1/2 by default. See [PINOUT.txt](PINOUT.txt) for the complete pin table, power requirements, and wiring details.

## Build and upload

Install VS Code with the PlatformIO extension, or PlatformIO Core. Open a terminal in the project root:

```sh
pio run -e blackpill_f401ce
pio run -e blackpill_f401ce -t upload --upload-protocol stlink
```

For ST-Link, use SWDIO=PA13, SWCLK=PA14, and common GND. PA11/PA12 are assigned to motor outputs, so do not use USB data with this pinout. If the PlatformIO CLI is not on PATH, open the PlatformIO terminal in VS Code.

To build the isolated servo PWM diagnostic firmware:

```sh
pio run -e blackpill_f401ce_servo_minimal
```

This environment builds a separate servo test program in place of the main control firmware.

## PC test tools

Install the dependencies with Python 3 and select your serial port (`COM7` is an example):

```sh
python -m pip install pyserial pygame
python tools/f401_test_gui.py --port COM7
python tools/f401_test_gui.py --web --port COM7 --auto-connect
python tools/f401_test_gui.py --dry-run
python tools/joystick_direct_f401.py --port COM7 --monitor-only
python tools/joystick_direct_f401.py --event-test
```

The desktop GUI uses Tkinter and falls back to a browser interface if Tkinter is unavailable. `pygame` is needed for gamepad support. A 3.3 V TTL USB-UART adapter can connect directly to the UART; cross TX/RX. Only one application should open a given serial port at a time.

## Basic usage

Commands are case-insensitive and accept CR/LF line endings:

```text
help
get mode
get sensors
get fault
params
mode arm
y
get status
stopall
mode safe
save
```

Manual drive commands include `forward <j> [pwm]`, `backward <j> [pwm]`, and `axis <j> <-1000..1000>`. `stopall` coast-stops all motors and disables servo PWM and the relays. `estop` also switches to SAFE mode. Settings update RAM first; to persist them, run `save` in SAFE mode with all motors stopped. Status telemetry is sent every 250 ms by default.

See [COMMANDS.txt](COMMANDS.txt) for all commands, servo calibration, breakpoints, elevator/drill control, and joystick mappings.

## Before operating

- Start testing with motor power disconnected, then test a single motor at low power. Stop the system and safely isolate power before changing ARM/DRILL wiring.
- **UART silence does not trigger an automatic stop on the F401.** The upper controller must detect communication loss and send `failsafe linkloss` / `failsafe timeout`; these commands cannot be delivered over a disconnected UART. `hb` is not a link watchdog.
- Do not power the servo from the BlackPill's 5 V/3.3 V pins. Use a suitable external servo supply and a common reference, keeping high-current return paths separate from the logic wiring.
- Follow the external motor pulldown and relay pull-up requirements in [PINOUT.txt](PINOUT.txt) for reset-time behavior. Do not press the board's PA0 KEY button while the servo is enabled.

## Files and checks

| Path | Contents |
| --- | --- |
| `src/main.cpp` | Main firmware, command handling, motor/sensor control, and safe-stop handling |
| `src/servo_minimal_test.cpp` | Isolated servo diagnostic firmware |
| `lib/servo_pwm/` | TIM5 servo PWM driver |
| `include/` | Breakpoint and reliability calculations |
| `tools/` | GUI, joystick bridge, and servo model checks |
| `test/` | C++ checks that can run on a host computer |
| `platformio.ini` | Board, build environments, and compiler flags |

Run the servo static/model regression checks:

```sh
python tools/test_servo_safety_model.py
```

These checks and a successful build do not replace physical verification of motors, braking, sensors, or power supplies.
