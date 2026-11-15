// Line protocol with abv_dock_bridge (see abv_dock_bridge/protocol.py).
//   Host -> Uno: "V,<closing m/s>,<gap m>"   "C,ARM|DISARM|LOCK|UNLOCK"
//   Uno -> Host: "S,<state>,<locked>,<hall1>,<hall2>,<estop>,<cycles>,<pwm>,<fault>"
//                "L,<free text>"
#pragma once
#include <Arduino.h>

enum class HostCommand : uint8_t { NONE, ARM, DISARM, LOCK, UNLOCK };

struct HostInput
{
    bool gotVelocity = false;     // a V line arrived this call
    float closing = 0.0f;         // m/s, + toward the target
    float gap = 0.0f;             // m, face gap
    HostCommand command = HostCommand::NONE;
};

namespace SerialProtocol
{
    void begin();
    // Reads whatever bytes are waiting; returns at most one parsed line per call.
    HostInput poll();
    void sendStatus(uint8_t aState, bool aLocked, bool aHall1, bool aHall2, bool aEstop,
                    uint16_t aCycles, uint8_t aPwm, uint8_t aFault);
    void log(const char* aText);
}
