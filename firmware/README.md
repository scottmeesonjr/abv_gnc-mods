# docking_controller (Arduino Uno)

Firmware for the docking mechanism. Talks to `abv_dock_bridge` over USB.

## Upload
1. Arduino IDE: open `docking_controller/docking_controller.ino`.
2. Library Manager: install **Adafruit INA219** (set `USE_INA219 0` in `Config.h` if the sensor isn't fitted).
3. Board: Arduino Uno. Upload.

## Before hardware tests
Every value marked `PLACEHOLDER` in `Config.h` must be checked against the stepper schematic:
pins, signal polarity, `DIR_TO_LOCK`, `STEP_RATE_HZ`, `MAX_TRAVEL_STEPS`, `STALL_CURRENT_MA`.

## Bench test without the ABV
Serial Monitor at 115200 baud, line ending "Newline". Type:

| Send | Expect |
|---|---|
| `C,ARM` | `L,state ARMED`, S lines with state 1 |
| (hold a magnet over both Hall sensors) | state 2 (CAPTURE), then 3 (LOCKED), `locked` = 1 |
| `C,UNLOCK` | state 4, then 0; cycle count +1 |
| `C,LOCK` | drives closed without the Hall sensors (locked stays 0 without the probe) |
| press E-stop | `L,E-stop pressed`, estop = 1, commands refused |
| `C,DISARM` | clears a FAULT |

Status line: `S,state,locked,hall1,hall2,estop,cycles,magnet_pwm,fault` at 20 Hz.
Fault codes: 1 stall (INA219), 2 limit switch not reached, 3 both limit switches pressed.

The link watchdog only starts after the first `V,` line, so hand-typed bench commands work.

## PC test (no Arduino needed)
`bash host_test/run_tests.sh` runs the state machine against a simulated latch (22 checks).
