// =====================================================================
//  SatDock docking mechanism controller (Arduino Uno)
//
//  Talks to abv_dock_bridge over USB (115200 baud, see SerialProtocol.h).
//  Mechanism states (match abv_msgs/DockStatus):
//    0 IDLE     latch open, not waiting for capture
//    1 ARMED    latch open, locks automatically when both Hall sensors see the probe
//    2 CAPTURE  stepper driving the latch closed
//    3 LOCKED   latch closed (reported "locked" only while both Hall sensors agree, SYS.05)
//    4 RELEASE  stepper driving the latch open
//    5 FAULT    actuator de-energized until DISARM clears it
//
//  Safety:
//  - E-stop (TEST.02): cuts actuator power in hardware; the Uno stays on (USB),
//    reports estop = 1, refuses motion, and stops any move in progress.
//  - Link watchdog: no V line for 200 ms (nav lost, cable pulled, bridge down)
//    -> magnet off and ARMED drops to IDLE, so nothing captures blind. A lock or
//    unlock already under way finishes (a half-closed latch is worse), and a
//    LOCKED latch is never released by a link loss.
//  - Stall (INA219 current spike) or a move that never reaches its limit
//    switch -> FAULT, actuator off.
// =====================================================================
#include "Config.h"
#include "SerialProtocol.h"
#include "Sensors.h"
#include "Actuator.h"
#include "Magnet.h"

enum MechState : uint8_t { IDLE = 0, ARMED = 1, CAPTURE = 2, LOCKED = 3, RELEASE = 4, FAULT = 5 };

static MechState gState = IDLE;
static uint8_t gFault = FAULT_NONE;
static uint16_t gCycles = 0;          // completed lock -> unlock cycles since boot (SYS.06)
static bool gReleaseFromLock = false;
static uint8_t gMagPwm = 0;

static bool gLinkEverUp = false;
static bool gLinkWasOk = false;
static unsigned long gLastVMs = 0;
static float gClosing = 0.0f;
static float gGap = 1.0f;

static unsigned long gStallSince = 0;
static bool gStallTiming = false;
static unsigned long gHallLostSince = 0;
static bool gHallLostTiming = false;
static bool gEstopWas = false;
static unsigned long gLastStatusMs = 0;

static const char* stateName(MechState s)
{
    switch (s)
    {
        case IDLE: return "IDLE";       case ARMED: return "ARMED";
        case CAPTURE: return "CAPTURE"; case LOCKED: return "LOCKED";
        case RELEASE: return "RELEASE"; default: return "FAULT";
    }
}

static void setState(MechState s)
{
    if (s == gState) return;
    gState = s;
    char buf[32];
    snprintf(buf, sizeof(buf), "state %s", stateName(s));
    SerialProtocol::log(buf);
}

static void enterFault(uint8_t aCode, const char* aWhy)
{
    Actuator::disable();
    Magnet::off();
    gMagPwm = 0;
    gFault = aCode;
    SerialProtocol::log(aWhy);
    setState(FAULT);
}

static void startLock(unsigned long aNowMs)
{
    Magnet::off();
    gMagPwm = 0;
    gStallTiming = false;
    Actuator::startLock(aNowMs);
    setState(CAPTURE);
}

static void startUnlock(const SensorReadings& s, unsigned long aNowMs)
{
    Magnet::off();
    gMagPwm = 0;
    gReleaseFromLock = (gState == LOCKED);
    if (s.limitUnlocked)
    {
        // already open (e.g. UNLOCK after a missed capture)
        Actuator::disable();
        setState(IDLE);
        return;
    }
    gStallTiming = false;
    Actuator::startUnlock(aNowMs);
    setState(RELEASE);
}

static void handleCommand(HostCommand c, const SensorReadings& s, unsigned long nowMs)
{
    if (c == HostCommand::NONE) return;

    if (c == HostCommand::DISARM)
    {
        if (gState == FAULT)
        {
            gFault = FAULT_NONE;
            SerialProtocol::log("fault cleared");
            setState(s.limitLocked ? LOCKED : IDLE);
        }
        else if (gState == ARMED)
        {
            Magnet::off();
            gMagPwm = 0;
            setState(IDLE);
        }
        else if (gState == CAPTURE)
        {
            Actuator::stop();
            setState(s.limitLocked ? LOCKED : IDLE);
        }
        // LOCKED / RELEASE / IDLE: DISARM never releases or interrupts a release
        return;
    }

    if (gState == FAULT)
    {
        SerialProtocol::log("in FAULT: send DISARM to clear");
        return;
    }
    if (s.estop)
    {
        SerialProtocol::log("E-stop pressed: command refused");
        return;
    }

    switch (c)
    {
        case HostCommand::ARM:
            if (gState == IDLE)
            {
                if (!s.limitUnlocked) SerialProtocol::log("warning: latch not at open limit");
                setState(ARMED);
            }
            break;
        case HostCommand::LOCK:
            if (gState == IDLE || gState == ARMED) startLock(nowMs);
            break;
        case HostCommand::UNLOCK:
            if (gState == LOCKED || gState == ARMED || gState == IDLE || gState == CAPTURE)
            {
                Actuator::stop();
                startUnlock(s, nowMs);
            }
            break;
        default:
            break;
    }
}

static void checkStall(const SensorReadings& s, unsigned long nowMs)
{
    if (!Actuator::isMoving() || !s.currentValid ||
        nowMs - Actuator::moveStartedMs() < MOVE_IGNORE_MS)
    {
        gStallTiming = false;
        return;
    }
    if (s.currentMa > STALL_CURRENT_MA)
    {
        if (!gStallTiming) { gStallTiming = true; gStallSince = nowMs; }
        else if (nowMs - gStallSince >= STALL_TIME_MS) enterFault(FAULT_STALL, "stall: actuator current too high");
    }
    else
    {
        gStallTiming = false;
    }
}

static void updateLed(unsigned long nowMs)
{
    bool on = false;
    switch (gState)
    {
        case LOCKED:  on = true; break;
        case ARMED:   on = (nowMs / 500) % 2; break;
        case FAULT:   on = (nowMs / 100) % 2; break;
        case CAPTURE: case RELEASE: on = (nowMs / 250) % 2; break;
        default:      on = false; break;
    }
    digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

void setup()
{
    pinMode(PIN_STATUS_LED, OUTPUT);
    SerialProtocol::begin();
    Actuator::begin();
    Magnet::begin();
    Sensors::begin();
    SerialProtocol::log("docking_controller ready");
}

void loop()
{
    const unsigned long nowMs = millis();
    const unsigned long nowUs = micros();

    // ---- host input ----
    HostInput in = SerialProtocol::poll();
    if (in.gotVelocity)
    {
        gLastVMs = nowMs;
        gLinkEverUp = true;
        gClosing = in.closing;
        gGap = in.gap;
    }
    const bool linkOk = gLinkEverUp && (nowMs - gLastVMs <= LINK_TIMEOUT_MS);

    SensorReadings s = Sensors::read(nowMs);

    // ---- wiring sanity ----
    if (gState != FAULT && s.limitLocked && s.limitUnlocked)
    {
        enterFault(FAULT_BOTH_LIMITS, "both limit switches pressed: check wiring");
    }

    // ---- E-stop (TEST.02) ----
    if (s.estop && !gEstopWas)
    {
        SerialProtocol::log("E-stop pressed");
        Actuator::disable();
        Magnet::off();
        gMagPwm = 0;
        if (gState == CAPTURE || gState == RELEASE || gState == ARMED)
        {
            // move interrupted: report where the latch actually is
            setState(s.limitLocked ? LOCKED : IDLE);
        }
    }
    else if (!s.estop && gEstopWas)
    {
        SerialProtocol::log("E-stop released");
    }
    gEstopWas = s.estop;

    // ---- link watchdog ----
    if (gLinkWasOk && !linkOk)
    {
        SerialProtocol::log("link lost: no V line for 200 ms");
        Magnet::off();
        gMagPwm = 0;
        if (gState == ARMED) setState(IDLE);
    }
    else if (!gLinkWasOk && linkOk)
    {
        SerialProtocol::log("link OK");
    }
    gLinkWasOk = linkOk;

    handleCommand(in.command, s, nowMs);

    // ---- state machine ----
    switch (gState)
    {
        case ARMED:
            if (!s.estop && s.bothHalls)
            {
                SerialProtocol::log("capture: both Hall sensors, locking");
                startLock(nowMs);
            }
            else
            {
                gMagPwm = Magnet::update(linkOk, gClosing, gGap);
            }
            break;

        case CAPTURE:
        {
            MoveResult r = Actuator::update(nowUs, s.limitLocked);
            if (r == MoveResult::DONE)
            {
                gHallLostTiming = false;
                setState(LOCKED);
            }
            else if (r == MoveResult::TRAVEL_FAULT)
            {
                enterFault(FAULT_TRAVEL, "lock limit switch not reached");
            }
            else
            {
                checkStall(s, nowMs);
            }
            break;
        }

        case LOCKED:
            // a Hall sensor dropping out while latched means the halves separated
            if (s.hall1 && s.hall2)
            {
                gHallLostTiming = false;
            }
            else if (!gHallLostTiming)
            {
                gHallLostTiming = true;
                gHallLostSince = nowMs;
            }
            else if (gHallLostSince != 0 && nowMs - gHallLostSince >= HALL_DEBOUNCE_MS)
            {
                // Reported as "not locked" (see below) rather than a FAULT, so
                // the host can still send UNLOCK and finish the cycle.
                SerialProtocol::log("warning: Hall sensors disagree while latched");
                gHallLostSince = 0;   // log once per drop-out
            }
            break;

        case RELEASE:
        {
            MoveResult r = Actuator::update(nowUs, s.limitUnlocked);
            if (r == MoveResult::DONE)
            {
                Actuator::disable();
                if (gReleaseFromLock) gCycles++;
                gReleaseFromLock = false;
                setState(IDLE);
            }
            else if (r == MoveResult::TRAVEL_FAULT)
            {
                enterFault(FAULT_TRAVEL, "unlock limit switch not reached");
            }
            else
            {
                checkStall(s, nowMs);
            }
            break;
        }

        default:
            break;
    }

    // "locked" is only reported while the latch is closed AND both Hall
    // sensors confirm the probe is seated (SYS.05)
    const bool locked = (gState == LOCKED) && s.hall1 && s.hall2;

    updateLed(nowMs);

    if (nowMs - gLastStatusMs >= STATUS_PERIOD_MS)
    {
        gLastStatusMs = nowMs;
        SerialProtocol::sendStatus(gState, locked, s.hall1, s.hall2, s.estop,
                                   gCycles, gMagPwm, gFault);
    }
}
