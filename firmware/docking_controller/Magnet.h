// Electromagnet soft capture / damping through an L293D (USE_ELECTROMAGNET 1).
#pragma once
#include <Arduino.h>

namespace Magnet
{
    void begin();
    // Called each loop while ARMED. Returns the PWM applied (0-255) for the S line.
    uint8_t update(bool aLinkOk, float aClosing, float aGap);
    void off();
}
