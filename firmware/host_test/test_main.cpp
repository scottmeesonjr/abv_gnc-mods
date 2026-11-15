// Host-side test of docking_controller with a simulated latch.
#include "Arduino.h"
#include "Config.h"
#include <cassert>
#include <iostream>
#include <vector>

namespace sim {
  unsigned long now_us = 0; int pin_in[32]; int pin_out[32]; int step_count_hi = 0;
  std::string tx; std::deque<char> rx; float current_ma = 100.0f; bool ina_present = true;
}
SerialMock Serial;
void digitalWrite(int p, int v) {
  if (p == PIN_STEP && v == HIGH && sim::pin_out[p] == LOW) sim::step_count_hi++;
  sim::pin_out[p] = v;
}
void setup(); void loop();

// --- latch model: position in steps, 0 = open, LATCH_TRAVEL = closed ---
static const int LATCH_TRAVEL = 1200;
static int latch = 0;
static int lastSteps = 0;
static bool lockSwitchBroken = false;

static void updateLatchModel() {
  int newSteps = sim::step_count_hi - lastSteps; lastSteps = sim::step_count_hi;
  bool enabled = sim::pin_out[PIN_ENABLE] == ENABLE_ACTIVE_LEVEL;
  if (enabled && newSteps) latch += (sim::pin_out[PIN_DIR] == DIR_TO_LOCK ? newSteps : -newSteps);
  if (latch < 0) latch = 0;
  if (latch > LATCH_TRAVEL + 50) latch = LATCH_TRAVEL + 50;
  bool atLock = latch >= LATCH_TRAVEL && !lockSwitchBroken;
  sim::pin_in[PIN_LIMIT_LOCKED] = atLock ? LIMIT_ACTIVE_LEVEL : !LIMIT_ACTIVE_LEVEL;
  sim::pin_in[PIN_LIMIT_UNLOCKED] = latch <= 0 ? LIMIT_ACTIVE_LEVEL : !LIMIT_ACTIVE_LEVEL;
}
static void halls(bool on) {
  sim::pin_in[PIN_HALL_1] = on ? HALL_ACTIVE_LEVEL : !HALL_ACTIVE_LEVEL;
  sim::pin_in[PIN_HALL_2] = on ? HALL_ACTIVE_LEVEL : !HALL_ACTIVE_LEVEL;
}
static void estop(bool pressed) { sim::pin_in[PIN_ESTOP_SENSE] = pressed ? !ESTOP_OK_LEVEL : ESTOP_OK_LEVEL; }
static void send(const char* line) { for (const char* c = line; *c; ++c) sim::rx.push_back(*c); }
static bool sendV = true;
// run for ms, 100 us per loop, V line every 40 ms if sendV
static void run(unsigned long ms) {
  unsigned long end = sim::now_us + ms * 1000UL;
  while (sim::now_us < end) {
    if (sendV && (sim::now_us % 40000UL) == 0) send("V,0.0750,0.0100\n");
    updateLatchModel(); loop(); sim::now_us += 100;
  }
}
// parse the last S line
struct Status { int state, locked, h1, h2, estop, cycles, pwm, fault; };
static Status last() {
  size_t p = sim::tx.rfind("S,"); assert(p != std::string::npos);
  Status s; sscanf(sim::tx.c_str() + p, "S,%d,%d,%d,%d,%d,%d,%d,%d", &s.state, &s.locked, &s.h1, &s.h2, &s.estop, &s.cycles, &s.pwm, &s.fault);
  return s;
}
static bool logged(const char* t) { return sim::tx.find(t) != std::string::npos; }
#define CHECK(c) do { if (!(c)) { std::cerr << "FAIL line " << __LINE__ << ": " #c << "\n" << sim::tx.substr(sim::tx.size() > 600 ? sim::tx.size() - 600 : 0) << "\n"; return 1; } ++passed; } while (0)

int main() {
  int passed = 0;
  halls(false); estop(false); updateLatchModel();
  setup(); run(100);
  CHECK(last().state == 0 && last().fault == 0);

  // 1. nominal: ARM -> capture on both Halls -> LOCKED -> UNLOCK -> IDLE, cycle counted
  send("C,ARM\n"); run(100); CHECK(last().state == 1);
  halls(true); run(10); CHECK(last().state == 1);           // not yet debounced
  run(100); CHECK(last().state == 2);                       // CAPTURE
  run(2000); CHECK(last().state == 3 && last().locked == 1); // 1200 steps at 800 Hz = 1.5 s
  send("C,UNLOCK\n"); run(100); CHECK(last().state == 4);
  halls(false); run(2000); CHECK(last().state == 0 && last().cycles == 1 && latch == 0);

  // 2. one Hall only: no capture
  send("C,ARM\n"); run(100);
  sim::pin_in[PIN_HALL_1] = HALL_ACTIVE_LEVEL; run(500); CHECK(last().state == 1);
  halls(false);

  // 3. link watchdog: V lines stop while ARMED -> IDLE
  sendV = false; run(300); CHECK(last().state == 0 && logged("link lost"));
  sendV = true; run(100); CHECK(logged("link OK"));

  // 4. E-stop during CAPTURE -> move stopped, commands refused, estop reported
  send("C,ARM\n"); run(100); halls(true); run(300); CHECK(last().state == 2);
  estop(true); run(100);
  CHECK(last().estop == 1 && last().state == 0 && sim::pin_out[PIN_ENABLE] != ENABLE_ACTIVE_LEVEL);
  send("C,LOCK\n"); run(100); CHECK(last().state == 0 && logged("command refused"));
  estop(false); run(100);
  send("C,UNLOCK\n"); run(2000); CHECK(last().state == 0 && latch == 0);   // reopens the half-closed latch
  halls(false); run(100);

  // 5. travel fault: lock switch never closes -> FAULT 2, DISARM clears, UNLOCK reopens
  lockSwitchBroken = true;
  send("C,LOCK\n"); run(6000); CHECK(last().state == 5 && last().fault == FAULT_TRAVEL);
  send("C,UNLOCK\n"); run(100); CHECK(last().state == 5);
  send("C,DISARM\n"); run(100); CHECK(last().state == 0 && last().fault == 0);
  send("C,UNLOCK\n"); run(4000); CHECK(last().state == 0 && latch == 0);
  lockSwitchBroken = false;

  // 6. stall: current spike while moving -> FAULT 1 (only with the INA219 fitted)
#if USE_INA219
  send("C,LOCK\n"); run(150); sim::current_ma = 2500.0f; run(300);
  CHECK(last().state == 5 && last().fault == FAULT_STALL);
  sim::current_ma = 100.0f; send("C,DISARM\n"); send("C,UNLOCK\n"); run(3000); CHECK(last().state == 0);
#endif

  // 7. garbage and over-long lines are ignored
  send("garbage\nV,abc,1\nC,DANCE\n"); send(std::string(200, 'x').c_str()); send("\n"); run(100);
  CHECK(last().state == 0 && logged("unknown command"));

  // 8. status rate ~20 Hz
  size_t before = std::count(sim::tx.begin(), sim::tx.end(), 'S'); (void)before;
  std::string mark = sim::tx; run(1000);
  int n = 0; for (size_t p = mark.size(); (p = sim::tx.find("\nS,", p)) != std::string::npos; ++p) ++n;
  CHECK(n >= 19 && n <= 21);

  std::cout << "all " << passed << " checks passed\n";
  return 0;
}
