// Minimal Arduino API stand-in so the firmware can run on a PC for testing.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <deque>
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define F(x) (x)
typedef uint8_t byte;
namespace sim {
  extern unsigned long now_us;
  extern int pin_in[32];      // levels the firmware reads
  extern int pin_out[32];     // levels the firmware wrote
  extern int step_count_hi;   // rising edges on STEP pin
  extern std::string tx;      // everything the firmware printed
  extern std::deque<char> rx; // bytes waiting to be read
  extern float current_ma;
  extern bool ina_present;
}
inline unsigned long millis() { return sim::now_us / 1000UL; }
inline unsigned long micros() { return sim::now_us; }
inline void delayMicroseconds(unsigned) {}
inline void pinMode(int, int) {}
inline int digitalRead(int p) { return sim::pin_in[p]; }
void digitalWrite(int p, int v);
inline void analogWrite(int p, int v) { sim::pin_out[p] = v; }
struct SerialMock {
  void begin(unsigned long) {}
  int available() { return (int)sim::rx.size(); }
  int read() { if (sim::rx.empty()) return -1; char c = sim::rx.front(); sim::rx.pop_front(); return (unsigned char)c; }
  void print(const char* s) { sim::tx += s; }
  void print(char c) { sim::tx += c; }
  void print(int v) { sim::tx += std::to_string(v); }
  void print(unsigned v) { sim::tx += std::to_string(v); }
  void print(long v) { sim::tx += std::to_string(v); }
  void print(unsigned long v) { sim::tx += std::to_string(v); }
};
extern SerialMock Serial;
