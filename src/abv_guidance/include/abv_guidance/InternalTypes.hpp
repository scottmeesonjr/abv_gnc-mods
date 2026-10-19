#ifndef INTERNALTYPES_HPP
#define INTERNALTYPES_HPP

#include <string> 
#include <Eigen/Dense>

struct Waypoint
{
    std::string mType;
    Eigen::Vector3d mPose;
    Eigen::Vector3d mVel;

    // Per-waypoint override (seconds) for how long the state machine will
    // wait for arrival before giving up on this waypoint. <= 0 means "use
    // the configured default" (GuidanceConfig::mWaypointTimeout).
    double mTimeout;

    // Per-waypoint, per-axis override for arrival tolerance [m, m, rad].
    // Each axis <= 0 means "no override for this axis" - falls back to the
    // enclosing Command's mArrivalTol, which itself falls back per-axis to
    // the configured default in GoalManager::setArrivalTolerance.
    Eigen::Vector3d mArrivalTol;

    Waypoint() : mTimeout(-1.0), mArrivalTol(-1.0, -1.0, -1.0) {}
    Waypoint(double x, double y, double yaw, const std::string& aType, double aTimeout = -1.0,
             const Eigen::Vector3d& aArrivalTol = Eigen::Vector3d(-1.0, -1.0, -1.0))
        : mType(aType), mPose(Eigen::Vector3d(x, y, yaw)), mTimeout(aTimeout), mArrivalTol(aArrivalTol)
    {;}

};

struct Command
{
    std::string mType; 
    double mDuration; 
    Waypoint mGoal;
    Eigen::Vector3d mArrivalTol;  
};

#endif 