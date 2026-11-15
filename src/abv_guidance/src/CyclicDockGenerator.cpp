
#include <algorithm>
#include <cmath>

#include "plog/Log.h"

#include "abv_msgs/msg/dock_command.hpp"
#include "abv_common/ConfigurationManager.h"

#include "abv_guidance/CyclicDockGenerator.h"

namespace
{
    // s, pause after UNLOCK so the latch is open before the vehicle backs off
    constexpr double kReleaseSettle_s = 1.0;

    // s, zero-velocity hold at the standoff point before each approach, so the
    // approach always starts from rest even if the standoff was only reached
    // within the relaxed (timed-out) tolerance while still drifting
    constexpr double kStandoffSettle_s = 3.0;

    // s, time allowed to reach the standoff point. With ~0.2 N of thrust on a
    // 12.7 kg vehicle, crossing the table and settling inside the tight
    // StartTolerance takes well over the 30 s default waypoint timeout.
    constexpr double kStandoffTimeout_s = 120.0;
}

CyclicDockGenerator::CyclicDockGenerator(double aRadialOffset, double aAngularOffset, double aApproachSpeed,
                                         int aCycles, const Scene& aScene) :
    mRadialOffset(aRadialOffset), mAngularOffset(aAngularOffset), mScene(aScene), mIndex(0)
{
    mConfig = ConfigurationManager::getInstance()->getDockingConfig();
    mApproachSpeed = (aApproachSpeed > 0.0) ? aApproachSpeed : mConfig.mApproachVelocity;
    mCycles = (aCycles > 0) ? aCycles : mConfig.mCycles;
}

CyclicDockGenerator::~CyclicDockGenerator()
{

}

bool CyclicDockGenerator::init()
{
    mPath.clear();
    mIndex = 0;

    // m/s, hard ceiling on a commanded approach speed. Runs above the 0.10 m/s
    // band are allowed (to exercise the 0.125 m/s alert) but not wildly so.
    constexpr double kMaxApproachSpeed = 0.15;
    if (mApproachSpeed > kMaxApproachSpeed)
    {
        LOGE << "Approach speed " << mApproachSpeed << " m/s is above the " << kMaxApproachSpeed
             << " m/s safety ceiling, refusing the run";
        return false;
    }
    if (mApproachSpeed < 0.05 || mApproachSpeed > 0.10)
    {
        LOGW << "Approach speed " << mApproachSpeed << " m/s is outside the SYS.10 band (0.05-0.10 m/s)";
    }

    if (mCycles < 1)
    {
        LOGW << "Cycle count is " << mCycles << ", nothing to run";
        return false;
    }

    auto standoff = DockingGeometry::chaserPoseAtGap(mConfig, mConfig.mStandoffDistance,
                                                     mRadialOffset, mAngularOffset);
    auto contact = DockingGeometry::chaserPoseAtGap(mConfig, 0.0, mRadialOffset, mAngularOffset);
    auto vel = DockingGeometry::approachVelocity(mConfig, mApproachSpeed);

    // Refuse to run if the standoff point would put the vehicle off the table.
    // This also catches a TargetPose that was never filled in.
    TableViewConfig& table = ConfigurationManager::getInstance()->getTableViewConfig();
    double halfExtent = 0.5 * std::max(table.mRobotWidth, table.mRobotLength);
    for (const auto& p : {standoff, contact})
    {
        if (!mScene.isWithinBounds(AxisAlignedBoundingBox::fromCircle(p[0], p[1], halfExtent)))
        {
            LOGE << "Docking pose (" << p[0] << ", " << p[1] << ") puts the vehicle outside the table bounds. "
                 << "Check Docking.TargetPose / PortOffset / StandoffDistance and the test offsets.";
            return false;
        }
    }

    Eigen::Vector3d startTol(mConfig.mStartTolerance[0], mConfig.mStartTolerance[1], mConfig.mStartTolerance[2]);
    Waypoint standoffWp(standoff[0], standoff[1], standoff[2], WaypointType::POSE, kStandoffTimeout_s, startTol);

    for (int cycle = 0; cycle < mCycles; cycle++)
    {
        Waypoint toStandoff = standoffWp;
        mPath.push_back(toStandoff.label(cycle == 0 ? "STANDOFF" : "BACK OFF"));
        mPath.push_back(Waypoint::hold(kStandoffSettle_s).label("SETTLE"));
        mPath.push_back(Waypoint::dockCommand(abv_msgs::msg::DockCommand::CMD_ARM).label("ARM"));
        mPath.push_back(Waypoint::approach(vel[0], vel[1], mConfig.mApproachTimeout,
                                           Eigen::Vector3d(contact[0], contact[1], contact[2])).label("APPROACH"));
        mPath.push_back(Waypoint::hold(mConfig.mDwellTime).label("LOCK"));
        mPath.push_back(Waypoint::dockCommand(abv_msgs::msg::DockCommand::CMD_UNLOCK).label("UNLOCK"));
        mPath.push_back(Waypoint::hold(kReleaseSettle_s).label("RELEASE"));
    }

    // back off clear of the target and leave the mechanism safe
    Waypoint finalBackOff = standoffWp;
    mPath.push_back(finalBackOff.label("BACK OFF"));
    mPath.push_back(Waypoint::dockCommand(abv_msgs::msg::DockCommand::CMD_DISARM).label("DISARM"));

    LOGI << "Docking run: " << mCycles << " cycles, radial offset " << mRadialOffset
         << " m, angular offset " << mAngularOffset << " rad, standoff ("
         << standoff[0] << ", " << standoff[1] << ", " << standoff[2] << "), approach "
         << mApproachSpeed << " m/s";
    return true;
}

bool CyclicDockGenerator::hasNext()
{
    return mIndex < mPath.size();
}

Waypoint CyclicDockGenerator::getNext()
{
    return mPath.at(mIndex++);
}

std::vector<Waypoint> CyclicDockGenerator::getPath() const
{
    return std::vector<Waypoint>(mPath.begin() + mIndex, mPath.end());
}

std::size_t CyclicDockGenerator::getPathPreviewLength() const
{
    // one cycle's worth of steps is enough to show the standoff and contact points
    return 8;
}
