#ifndef VELOCITYALERTMONITOR_H
#define VELOCITYALERTMONITOR_H

#include <string>

#include "abv_common/AbvState.hpp"
#include "abv_common/Configurations.h"

// SYS.01 velocity alert. Fed every navigation cycle by VehicleStateTracker;
// publishes abv_msgs/NavAlert on abv/nav/alert with speed, closing velocity
// and face gap to the docking target, plus the alert flag.
//
// The alert compares total planar speed, sqrt(vx^2 + vy^2), not closing
// velocity, so it also catches a fast lateral drift. It raises above
// Navigation.VelocityAlertThreshold and clears below VelocityAlertClear.
class VelocityAlertMonitor
{
public:
    VelocityAlertMonitor();
    ~VelocityAlertMonitor();

    // aValid = false (nav dropout): velocities can't be trusted, so the alert
    // keeps its previous value instead of clearing on the zeroed velocity.
    void update(const AbvState& aState, bool aValid);

    // Pure hysteresis rule, no ROS, so it can be unit tested on its own.
    static bool nextAlertState(bool aCurrentAlert, double aSpeed,
                               double aThreshold, double aClear);

private:
    std::string mTopicName;
    double mThreshold;
    double mClear;
    DockingConfig mDockingConfig;
    bool mAlert;
};

#endif // VELOCITYALERTMONITOR_H
