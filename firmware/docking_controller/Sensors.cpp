#include "Sensors.h"
#include "Config.h"
#include "SerialProtocol.h"

#if USE_INA219
#include <Wire.h>
#include <Adafruit_INA219.h>
namespace { Adafruit_INA219 sIna; bool sInaOk = false; }
#endif

namespace
{
    unsigned long sBothSince = 0;
    bool sBothPrev = false;
}

namespace Sensors
{
    void begin()
    {
        pinMode(PIN_HALL_1, INPUT_PULLUP);
        pinMode(PIN_HALL_2, INPUT_PULLUP);
        pinMode(PIN_LIMIT_LOCKED, INPUT_PULLUP);
        pinMode(PIN_LIMIT_UNLOCKED, INPUT_PULLUP);
        pinMode(PIN_ESTOP_SENSE, INPUT_PULLUP);
#if USE_INA219
        sInaOk = sIna.begin();
        if (!sInaOk) SerialProtocol::log("INA219 not found: stall detection off");
#endif
    }

    SensorReadings read(unsigned long aNowMs)
    {
        SensorReadings r;
        r.hall1 = digitalRead(PIN_HALL_1) == HALL_ACTIVE_LEVEL;
        r.hall2 = digitalRead(PIN_HALL_2) == HALL_ACTIVE_LEVEL;
        r.limitLocked = digitalRead(PIN_LIMIT_LOCKED) == LIMIT_ACTIVE_LEVEL;
        r.limitUnlocked = digitalRead(PIN_LIMIT_UNLOCKED) == LIMIT_ACTIVE_LEVEL;
        r.estop = digitalRead(PIN_ESTOP_SENSE) != ESTOP_OK_LEVEL;

        // Both Hall sensors must agree continuously for HALL_DEBOUNCE_MS (SYS.05)
        bool both = r.hall1 && r.hall2;
        if (both && !sBothPrev) sBothSince = aNowMs;
        sBothPrev = both;
        r.bothHalls = both && (aNowMs - sBothSince >= HALL_DEBOUNCE_MS);

#if USE_INA219
        if (sInaOk)
        {
            r.currentMa = sIna.getCurrent_mA();
            r.currentValid = true;
        }
#endif
        return r;
    }
}
