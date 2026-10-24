#ifndef INTERNALTYPES_HPP
#define INTERNALTYPES_HPP

#include <cstdint>
#include <string> 
#include <Eigen/Dense>

// Waypoint::mType values. "pose" is the original behavior; the rest were
// added for docking (see StateMachine::sendWaypoint for how each runs).
namespace WaypointType
{
    // drive to mPose with the position controller, done on controller ARRIVED
    inline const std::string POSE = "pose";
    // command velocity mVel (table frame) for mTimeout seconds, then stop
    inline const std::string VELOCITY = "vel";
    // docking approach: command velocity mVel (table frame) until the
    // mechanism reports lock, the face gap reaches Docking.CaptureRange,
    // or mTimeout runs out; then stop. mPose holds the contact pose (display only).
    inline const std::string APPROACH = "approach";
    // stop thrusting and wait mTimeout seconds (e.g. dwell while docked)
    inline const std::string HOLD = "hold";
    // send mDockCommand to the docking mechanism, then continue immediately
    inline const std::string DOCK_COMMAND = "dock_cmd";
}

// AbvControllerCommand.type string the controller uses for velocity commands.
// CHECK: must match what abv_commander's `vel` command sends.
inline const std::string kControllerVelocityType = "velocity";

struct Waypoint
{
    std::string mType;
    Eigen::Vector3d mPose;
    Eigen::Vector3d mVel;   // [vx, vy, wz] table frame, for VELOCITY / APPROACH

    // Per-waypoint override (seconds) for how long the state machine will
    // wait for arrival before giving up on this waypoint. <= 0 means "use
    // the configured default" (GuidanceConfig::mWaypointTimeout).
    // For VELOCITY / APPROACH / HOLD this is the step's duration or time limit.
    double mTimeout;

    // Per-waypoint, per-axis override for arrival tolerance [m, m, rad].
    // Each axis <= 0 means "no override for this axis" - falls back to the
    // enclosing Command's mArrivalTol, which itself falls back per-axis to
    // the configured default in GoalManager::setArrivalTolerance.
    Eigen::Vector3d mArrivalTol;

    // DOCK_COMMAND only: abv_msgs/DockCommand code (CMD_ARM, CMD_UNLOCK, ...)
    std::uint8_t mDockCommand;

    Waypoint() : mType(WaypointType::POSE), mPose(Eigen::Vector3d::Zero()), mVel(Eigen::Vector3d::Zero()),
                 mTimeout(-1.0), mArrivalTol(-1.0, -1.0, -1.0), mDockCommand(0) {}
    Waypoint(double x, double y, double yaw, const std::string& aType, double aTimeout = -1.0,
             const Eigen::Vector3d& aArrivalTol = Eigen::Vector3d(-1.0, -1.0, -1.0))
        : mType(aType), mPose(Eigen::Vector3d(x, y, yaw)), mVel(Eigen::Vector3d::Zero()),
          mTimeout(aTimeout), mArrivalTol(aArrivalTol), mDockCommand(0)
    {;}

    static Waypoint velocity(double aVx, double aVy, double aWz, double aDuration_s)
    {
        Waypoint wp;
        wp.mType = WaypointType::VELOCITY;
        wp.mVel << aVx, aVy, aWz;
        wp.mTimeout = aDuration_s;
        return wp;
    }

    static Waypoint approach(double aVx, double aVy, double aTimeout_s,
                             const Eigen::Vector3d& aContactPose)
    {
        Waypoint wp;
        wp.mType = WaypointType::APPROACH;
        wp.mVel << aVx, aVy, 0.0;
        wp.mTimeout = aTimeout_s;
        wp.mPose = aContactPose;
        return wp;
    }

    static Waypoint hold(double aDuration_s)
    {
        Waypoint wp;
        wp.mType = WaypointType::HOLD;
        wp.mTimeout = aDuration_s;
        return wp;
    }

    static Waypoint dockCommand(std::uint8_t aCode)
    {
        Waypoint wp;
        wp.mType = WaypointType::DOCK_COMMAND;
        wp.mDockCommand = aCode;
        return wp;
    }

    // true for waypoint types that have a meaningful mPose to draw in the GUI
    bool hasDisplayPose() const
    {
        return mType == WaypointType::POSE || mType == WaypointType::APPROACH;
    }
};

struct Command
{
    std::string mType; 
    double mDuration; 
    Waypoint mGoal;
    Eigen::Vector3d mArrivalTol;  
};

#endif 
