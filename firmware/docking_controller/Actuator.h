// Latch stepper through a STEP / DIR / EN driver (A4988, DRV8825, TMC2209, ...).
// Non-blocking: call update() every loop; it emits at most one step per call.
#pragma once
#include <Arduino.h>

enum class MoveResult : uint8_t { MOVING, DONE, TRAVEL_FAULT, IDLE };

namespace Actuator
{
    void begin();
    void startLock(unsigned long aNowMs);    // drive toward the locked limit switch
    void startUnlock(unsigned long aNowMs);  // drive toward the unlocked limit switch
    void stop();                             // stop now, driver disabled (unless holding)
    void disable();                          // stop now and always de-energize
    // aAtTarget: the limit switch for the current move is pressed
    MoveResult update(unsigned long aNowUs, bool aAtTarget);
    bool isMoving();
    unsigned long moveStartedMs();
}
