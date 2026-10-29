#pragma once

#include <array>
#include <cmath>
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

    // abv_simulator only: put a solid target face at TargetPose and fake the
    // mechanism (ARM / lock on contact / UNLOCK) on abv/dock/status, so docking
    // runs can be tested end to end in sim. Ignored on the real vehicle.
    bool mSimulateContact = false;

    // m, half the width of the passive face. The sim only treats the vehicle
    // as touching the target when its face center is within this distance of
    // the target center sideways.
    double mTargetHalfWidth = 0.15;
};

// Docking geometry shared by navigation (velocity alert) and guidance
// (standoff poses, approach velocity, stop-at-capture-range check).
//
// Conventions:
//   n = (cos(psi), sin(psi)) is the passive face's outward normal (psi = TargetPose[2]).
//   t = (-sin(psi), cos(psi)) is the lateral direction across the face.
//   The ABV's active face sits PortOffset ahead of its center along its +x body axis,
//   so a perfectly aligned ABV points at yaw psi + pi.
namespace DockingGeometry
{
    inline double wrapAngle(double aAngle)
    {
        return std::atan2(std::sin(aAngle), std::cos(aAngle));
    }

    // m, gap between the ABV's active face and the passive face, measured along n.
    // Positive = still apart, ~0 = contact, negative = past the face plane.
    inline double faceGap(const DockingConfig& aCfg, double aX, double aY, double aYaw)
    {
        const double psi = aCfg.mTargetPose[2];
        const double faceX = aX + aCfg.mPortOffset * std::cos(aYaw);
        const double faceY = aY + aCfg.mPortOffset * std::sin(aYaw);
        return (faceX - aCfg.mTargetPose[0]) * std::cos(psi) +
               (faceY - aCfg.mTargetPose[1]) * std::sin(psi);
    }

    // m, sideways offset of the ABV's active face center from the target center,
    // measured along t. Used by the sim to decide whether the faces overlap.
    inline double faceLateralOffset(const DockingConfig& aCfg, double aX, double aY, double aYaw)
    {
        const double psi = aCfg.mTargetPose[2];
        const double faceX = aX + aCfg.mPortOffset * std::cos(aYaw);
        const double faceY = aY + aCfg.mPortOffset * std::sin(aYaw);
        return (faceX - aCfg.mTargetPose[0]) * -std::sin(psi) +
               (faceY - aCfg.mTargetPose[1]) * std::cos(psi);
    }

    // m/s, velocity along the docking axis. Positive = moving toward the target.
    inline double closingVelocity(const DockingConfig& aCfg, double aVx, double aVy)
    {
        const double psi = aCfg.mTargetPose[2];
        return -(aVx * std::cos(psi) + aVy * std::sin(psi));
    }

    // [x, y, yaw] of the ABV center when its face is aGap from the target face,
    // shifted aRadialOffset (m) along t and rotated aAngularOffset (rad).
    // aGap = StandoffDistance gives the standoff pose; aGap = 0 gives the contact pose.
    inline std::array<double, 3> chaserPoseAtGap(const DockingConfig& aCfg, double aGap,
                                                 double aRadialOffset = 0.0,
                                                 double aAngularOffset = 0.0)
    {
        const double psi = aCfg.mTargetPose[2];
        const double nx = std::cos(psi), ny = std::sin(psi);
        const double tx = -ny, ty = nx;
        const double along = aGap + aCfg.mPortOffset;
        return {aCfg.mTargetPose[0] + nx * along + tx * aRadialOffset,
                aCfg.mTargetPose[1] + ny * along + ty * aRadialOffset,
                wrapAngle(psi + M_PI + aAngularOffset)};
    }

    // [vx, vy] in the table frame for a pure axial approach at aSpeed (m/s).
    inline std::array<double, 2> approachVelocity(const DockingConfig& aCfg, double aSpeed)
    {
        const double psi = aCfg.mTargetPose[2];
        return {-aSpeed * std::cos(psi), -aSpeed * std::sin(psi)};
    }
}
