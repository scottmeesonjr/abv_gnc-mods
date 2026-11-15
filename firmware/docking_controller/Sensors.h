// Hall-effect sensors, limit switches, E-stop sense and the INA219.
#pragma once
#include <Arduino.h>

struct SensorReadings
{
    bool hall1 = false;          // debounced: magnet seen
    bool hall2 = false;
    bool bothHalls = false;      // both seen for HALL_DEBOUNCE_MS (capture / lock confirm)
    bool limitLocked = false;
    bool limitUnlocked = false;
    bool estop = false;          // true = E-stop pressed (or sense wire broken)
    bool currentValid = false;   // INA219 present and read
    float currentMa = 0.0f;
};

namespace Sensors
{
    void begin();
    SensorReadings read(unsigned long aNowMs);
}
