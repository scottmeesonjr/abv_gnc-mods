// =====================================================================
//  docking_controller: build options, pins and tuning
//
//  Everything marked PLACEHOLDER must be checked against the final
//  stepper wiring and bench measurements before hardware tests.
// =====================================================================
#pragma once

// ---------------- Build options ----------------
// 1 = electromagnet soft-capture / damping module (Magnet.cpp) is used.
// The current block diagram has no electromagnet, so it is off by default.
#ifndef USE_ELECTROMAGNET
#define USE_ELECTROMAGNET 0
#endif

// 1 = INA219 current sensor fitted (stall detection). If it is not found
// at boot, the firmware logs a warning and runs without stall detection.
#ifndef USE_INA219
#define USE_INA219 1
#endif

// ---------------- Serial link (must match abv_dock_bridge) ----------------
#define SERIAL_BAUD            115200UL
#define STATUS_PERIOD_MS       50UL     // S line rate: 20 Hz
#define LINK_TIMEOUT_MS        200UL    // no V line for this long -> safe state

// ---------------- Pins: Arduino Uno (PLACEHOLDER, confirm vs. schematic) -----
// D0/D1 = USB serial, A4/A5 = I2C (INA219). Do not reuse them.
#define PIN_STEP               2        // stepper driver STEP
#define PIN_DIR                3        // stepper driver DIR
#define PIN_ENABLE             4        // stepper driver EN
#define PIN_HALL_1             6        // A1104 #1 (open drain, LOW = magnet present)
#define PIN_HALL_2             7        // A1104 #2
#define PIN_LIMIT_LOCKED       8        // SW1, latch fully locked (switch to GND, LOW = pressed)
#define PIN_LIMIT_UNLOCKED     9        // SW2, latch fully open
#define PIN_ESTOP_SENSE        10       // E-stop sense contact, see ESTOP_OK_LEVEL
#define PIN_STATUS_LED         13

// Electromagnet through an L293D (only with USE_ELECTROMAGNET 1)
#define PIN_MAG_PWM            5        // L293D EN1 (PWM)
#define PIN_MAG_IN1            11
#define PIN_MAG_IN2            12

// ---------------- Signal polarity (PLACEHOLDER) ----------------
#define HALL_ACTIVE_LEVEL      LOW      // A1104 pulls low when it sees the magnet
#define LIMIT_ACTIVE_LEVEL     LOW      // switch to GND with INPUT_PULLUP
#define ENABLE_ACTIVE_LEVEL    LOW      // A4988 / DRV8825 / TMC2209: EN is active low
// E-stop sense: a normally-closed contact to GND reads LOW while the E-stop is
// released. Pressing it (or a broken wire) reads HIGH through the pull-up,
// so a wiring fault fails safe.
#define ESTOP_OK_LEVEL         LOW

// ---------------- Latch motion (PLACEHOLDER, set on the bench) ----------------
#define DIR_TO_LOCK            HIGH     // DIR level that drives the latch toward locked
#define STEP_RATE_HZ           800      // steps per second while moving
#define MAX_TRAVEL_STEPS       4000     // a move that has not hit its limit switch by
                                        // now is a fault (jam, missing switch)
#define HOLD_WHEN_LOCKED       0        // 1 = keep the driver energized while locked

// ---------------- Stall detection (INA219, PLACEHOLDER) ----------------
#define STALL_CURRENT_MA       1500.0f  // actuator current that counts as a stall
#define STALL_TIME_MS          150UL    // must stay above it this long
#define MOVE_IGNORE_MS         100UL    // ignore the start-up current spike

// ---------------- Capture ----------------
#define HALL_DEBOUNCE_MS       30UL     // both Hall sensors must agree this long

// ---------------- Electromagnet (PLACEHOLDER, bench calibration) ----------------
// The field only acts in the last few millimeters, so the magnet only runs
// inside MAG_ACTIVE_GAP_M. Above MAG_BRAKE_ABOVE_M_S it repels to slow the
// approach (damping, DOCK.04); otherwise it attracts for soft capture (DOCK.03).
#define MAG_ACTIVE_GAP_M       0.005f
#define MAG_BRAKE_ABOVE_M_S    0.10f
#define MAG_PWM_ATTRACT        180
#define MAG_PWM_REPEL          255

// ---------------- Fault codes (reported in the S line) ----------------
#define FAULT_NONE             0
#define FAULT_STALL            1        // INA219 current above the stall limit
#define FAULT_TRAVEL           2        // limit switch not reached in MAX_TRAVEL_STEPS
#define FAULT_BOTH_LIMITS      3        // both limit switches pressed (wiring fault)
