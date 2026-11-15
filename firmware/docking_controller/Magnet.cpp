#include "Magnet.h"
#include "Config.h"

namespace
{
    void drive(int aDirection, uint8_t aPwm)  // +1 attract, -1 repel, 0 off
    {
#if USE_ELECTROMAGNET
        digitalWrite(PIN_MAG_IN1, aDirection > 0 ? HIGH : LOW);
        digitalWrite(PIN_MAG_IN2, aDirection < 0 ? HIGH : LOW);
        analogWrite(PIN_MAG_PWM, aDirection == 0 ? 0 : aPwm);
#else
        (void)aDirection; (void)aPwm;
#endif
    }
}

namespace Magnet
{
    void begin()
    {
#if USE_ELECTROMAGNET
        pinMode(PIN_MAG_IN1, OUTPUT);
        pinMode(PIN_MAG_IN2, OUTPUT);
        pinMode(PIN_MAG_PWM, OUTPUT);
#endif
        drive(0, 0);
    }

    uint8_t update(bool aLinkOk, float aClosing, float aGap)
    {
#if USE_ELECTROMAGNET
        // Without fresh nav data the gap is unknown: leave the magnet off.
        if (!aLinkOk || aGap > MAG_ACTIVE_GAP_M)
        {
            drive(0, 0);
            return 0;
        }
        if (aClosing > MAG_BRAKE_ABOVE_M_S)
        {
            drive(-1, MAG_PWM_REPEL);   // too fast: push back (damping)
            return MAG_PWM_REPEL;
        }
        drive(+1, MAG_PWM_ATTRACT);     // in band: pull in (soft capture)
        return MAG_PWM_ATTRACT;
#else
        (void)aLinkOk; (void)aClosing; (void)aGap;
        return 0;
#endif
    }

    void off() { drive(0, 0); }
}
