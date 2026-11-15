#pragma once
#include "Arduino.h"
struct Adafruit_INA219 { bool begin() { return sim::ina_present; } float getCurrent_mA() { return sim::current_ma; } };
