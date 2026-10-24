
#include <cmath>

#include "abv_common/ConfigurationManager.h"
#include "abv_common/RosTopicManager.h"
#include "abv_msgs/msg/nav_alert.hpp"
#include "plog/Log.h"

#include "abv_navigation/VelocityAlertMonitor.h"

VelocityAlertMonitor::VelocityAlertMonitor() :
    mTopicName("abv/nav/alert"), mAlert(false)
{
    NavigationConfig& navConfig = ConfigurationManager::getInstance()->getNavigationConfig();
    mThreshold = navConfig.mVelocityAlertThreshold;
    mClear = navConfig.mVelocityAlertClear;
    mDockingConfig = ConfigurationManager::getInstance()->getDockingConfig();

    RosTopicManager::getInstance()->createPublisher<abv_msgs::msg::NavAlert>(mTopicName);

    LOGD << "Velocity alert armed: raise above " << mThreshold
         << " m/s, clear below " << mClear << " m/s";
}

VelocityAlertMonitor::~VelocityAlertMonitor()
{

}

bool VelocityAlertMonitor::nextAlertState(bool aCurrentAlert, double aSpeed,
                                          double aThreshold, double aClear)
{
    if (!aCurrentAlert && aSpeed > aThreshold)
    {
        return true;
    }
    if (aCurrentAlert && aSpeed < aClear)
    {
        return false;
    }
    return aCurrentAlert;
}

void VelocityAlertMonitor::update(const AbvState& aState, bool aValid)
{
    const double speed = std::hypot(aState.vx, aState.vy);
    const double closing = DockingGeometry::closingVelocity(mDockingConfig, aState.vx, aState.vy);
    const double gap = DockingGeometry::faceGap(mDockingConfig, aState.x, aState.y, aState.theta);

    if (aValid)
    {
        const bool newAlert = nextAlertState(mAlert, speed, mThreshold, mClear);
        if (newAlert && !mAlert)
        {
            LOGW << "VELOCITY ALERT: speed " << speed << " m/s exceeds "
                 << mThreshold << " m/s (closing " << closing << " m/s, gap " << gap << " m)";
        }
        else if (!newAlert && mAlert)
        {
            LOGI << "Velocity alert cleared: speed " << speed << " m/s";
        }
        mAlert = newAlert;
    }

    abv_msgs::msg::NavAlert msg;
    msg.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    msg.speed = speed;
    msg.closing_velocity = closing;
    msg.range = gap;
    msg.alert = mAlert;

    RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::NavAlert>(mTopicName, msg);
}
