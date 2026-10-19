#ifndef VEHICLESTATETRACKER_H
#define VEHICLESTATETRACKER_H

#include <memory>
#include <thread> 

#include "abv_navigation/IStateFetcher.h"
#include "abv_navigation/StampedAbvState.h"
#include "abv_common/Configurations.h"
#include "abv_common/ConfigurationManager.h"
#include "abv_common/ConsumableBuffer.hpp"
#include "RosStatePublisher.h"
#include "ExtendedKalmanFilter.h"

#include "abv_msgs/msg/abv_thruster_status.hpp"

class VehicleStateTracker
{
public:
    VehicleStateTracker();
    ~VehicleStateTracker();

    enum class FetcherType
    {
        SIMULATED, 
        OPTITRACK, 
        VICON, 
        NUM_TYPES
    };

    void stateTrackerLoop(); 

    bool doStateTracking() {std::lock_guard<std::mutex> lock(mStateTrackingMutex); return mDoStateTracking; }
    void setStateTracking(bool aFlag) {std::lock_guard<std::mutex> lock(mStateTrackingMutex); mDoStateTracking = aFlag; }

private:

    // construct buffer first
    ConsumableBuffer<StampedAbvState> mStateBuffer;

    // time of the last real measurement update, and whether the estimate is
    // still within the configured dead-reckoning bound of that measurement
    std::chrono::system_clock::time_point mLastMeasurementTime;
    bool mIsNavValid;

    // polymorphic state fetcher interface so we arent tied to optiTrack
    std::shared_ptr<IStateFetcher> mStateFetcher; // state fetcher interface class 
    RosStatePublisher mStatePublisher; 
    NavigationConfig mConfig; 

    // TODO: add subscription to ControllerStatus, pass latest input to EKF
    ExtendedKalmanFilter mEKF; 

    bool mDoStateTracking;

    std::mutex mStateTrackingMutex; 
    std::thread mStateTrackingThread;  

    VehicleStateTracker::FetcherType toEnum(std::string aTrackerType);

    std::mutex mControlInputMutex; 
    Eigen::Vector3d mLatestControlInput; 
    void thrusterStatusCallback(abv_msgs::msg::AbvThrusterStatus::ConstSharedPtr aMsg);
  
    // Pure helper (no ROS/threads) so the dead-reckoning bound check can be unit
    // tested directly with fabricated time_points.
    inline bool isWithinDeadReckonBound(std::chrono::system_clock::time_point aNow,
                                        std::chrono::system_clock::time_point aLastMeasurementTime,
                                        double aBoundDuration_s);
};
#endif // VEHICLESTATETRACKER_H