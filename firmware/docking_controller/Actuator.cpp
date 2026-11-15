#include "Actuator.h"
#include "Config.h"

namespace
{
    const unsigned long kStepPeriodUs = 1000000UL / STEP_RATE_HZ;
    bool sMoving = false;
    bool sToLock = false;
    long sSteps = 0;
    unsigned long sLastStepUs = 0;
    unsigned long sStartMs = 0;

    void enableDriver(bool aOn)
    {
        digitalWrite(PIN_ENABLE, aOn ? ENABLE_ACTIVE_LEVEL : !ENABLE_ACTIVE_LEVEL);
    }

    void start(bool aToLock, unsigned long aNowMs)
    {
        sToLock = aToLock;
        digitalWrite(PIN_DIR, aToLock ? DIR_TO_LOCK : !DIR_TO_LOCK);
        enableDriver(true);
        sSteps = 0;
        sMoving = true;
        sStartMs = aNowMs;
        sLastStepUs = micros();
    }
}

namespace Actuator
{
    void begin()
    {
        pinMode(PIN_STEP, OUTPUT);
        pinMode(PIN_DIR, OUTPUT);
        pinMode(PIN_ENABLE, OUTPUT);
        digitalWrite(PIN_STEP, LOW);
        enableDriver(false);
    }

    void startLock(unsigned long aNowMs)   { start(true, aNowMs); }
    void startUnlock(unsigned long aNowMs) { start(false, aNowMs); }

    void stop()
    {
        sMoving = false;
        // keep holding torque only after a completed lock, if configured
        enableDriver(HOLD_WHEN_LOCKED && sToLock);
    }

    void disable()
    {
        sMoving = false;
        enableDriver(false);
    }

    MoveResult update(unsigned long aNowUs, bool aAtTarget)
    {
        if (!sMoving) return MoveResult::IDLE;
        if (aAtTarget)
        {
            stop();
            return MoveResult::DONE;
        }
        if (sSteps >= MAX_TRAVEL_STEPS)
        {
            disable();
            return MoveResult::TRAVEL_FAULT;
        }
        if (aNowUs - sLastStepUs >= kStepPeriodUs)
        {
            sLastStepUs = aNowUs;
            digitalWrite(PIN_STEP, HIGH);
            delayMicroseconds(3);   // >= 1.9 us pulse for DRV8825, 1 us for A4988
            digitalWrite(PIN_STEP, LOW);
            sSteps++;
        }
        return MoveResult::MOVING;
    }

    bool isMoving() { return sMoving; }
    unsigned long moveStartedMs() { return sStartMs; }
}
