#pragma once

#include <array>
#include <string>

// Docking campaign parameters, parsed from the Docking: block of config.yaml.
// Frame: OptiTrack table frame (x, y in m, yaw in rad).
struct DockingConfig
{
    // Passive target face: center point and outward-facing normal direction.
    // The ABV approaches along -normal, so the docking axis is set by TargetPose[2].
    std::array<double, 3> mTargetPose{0.0, 0.0, 0.0};

    // m, distance from the ABV's tracked center (OptiTrack rigid body origin)
    // to the face of the active docking half, along the ABV's +x body axis.
    double mPortOffset = 0.30;

    // m, face-to-face gap at the start of each approach (standoff point).
    double mStandoffDistance = 0.50;

    // m/s, commanded closing velocity for the approach (SYS.10 band: 0.05-0.10).
    double mApproachVelocity = 0.075;

    // s, approach is aborted (no capture) if lock isn't reported within this time.
    double mApproachTimeout = 30.0;

    // m, face gap at which the approach stops if no lock has been reported.
    double mCaptureRange = 0.01;

    // [m, m, rad], arrival tolerance at the standoff point (TEST.10 / TEST.11).
    std::array<double, 3> mStartTolerance{0.015, 0.015, 0.017};

    // s, time held docked before UNLOCK is sent.
    double mDwellTime = 5.0;

    // Engagement cycles per run (SYS.02).
    int mCycles = 20;

    // Arduino link (used by abv_dock_bridge).
    std::string mSerialPort = "/dev/ttyACM0";
    int mBaudRate = 115200;

    // s, DockStatus.link_ok goes false if no status line arrives within this time.
    double mStatusTimeout = 0.5;
};
