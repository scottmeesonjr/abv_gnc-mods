
#include "abv_navigation/VehicleStateTracker.h"
#include "abv_navigation/OptitrackStateFetcher_LibMocap.h"
#include "abv_navigation/SimulatedStateFetcher.h"
#include "abv_common/RateController.hpp"
#include "abv_common/DataLogger.h"
#include "abv_common/RosTopicManager.h"
#include "plog/Log.h"
#include <thread>

using OptitrackStateFetcher = OptitrackStateFetcher_LibMocap; 

VehicleStateTracker::VehicleStateTracker() : 
    mConfig(ConfigurationManager::getInstance()->getNavigationConfig())
{
    FetcherType typeToMake = toEnum(mConfig.mInterface); 

    switch (typeToMake)
    {
        case FetcherType::SIMULATED:
            
            mStateFetcher = std::make_shared<SimulatedStateFetcher>(mStateBuffer); 
            LOGD << "Configuring ABV to use Simulated state data"; 
            
            break;
        case FetcherType::OPTITRACK: 
            
            mStateFetcher = std::make_shared<OptitrackStateFetcher>(mStateBuffer, 
                                                                    mConfig.mServerIp, 
                                                                    mConfig.mLocalIp, 
                                                                    mConfig.mRigidBodyName); 
            LOGD << "Configuring ABV to use OptiTrack for state data"; 
            
            break;
        default:
            break;
    }

    setStateTracking(false); 
    mStateTrackingThread = std::thread(&VehicleStateTracker::stateTrackerLoop, this); 

    // setup subscriber to get applied thrust, used as the EKF's control input.
    // Lives on abv/thrusters/status (published by abv_thrusters) rather than
    // abv/controller/status, since applied thrust is computed by whichever
    // node is currently driving the thrusters, not abv_goal_manager.
    RosTopicManager::getInstance()->createSubscriber<abv_msgs::msg::AbvThrusterStatus>("abv/thrusters/status",
                                                                                          std::bind(&VehicleStateTracker::thrusterStatusCallback,
                                                                                          this,
                                                                                          std::placeholders::_1));
}

VehicleStateTracker::~VehicleStateTracker()
{
    setStateTracking(false); 

    if(mStateTrackingThread.joinable())
    {
        LOGD << "Joining state tracking thread"; 
        mStateTrackingThread.join(); 
    }

}

VehicleStateTracker::FetcherType VehicleStateTracker::toEnum(std::string aTrackerType)
{
    VehicleStateTracker::FetcherType enumToReturn; 

    if("Simulated" == aTrackerType || "simulated" == aTrackerType)
    {
        enumToReturn = VehicleStateTracker::FetcherType::SIMULATED; 
    }
    else if ("Optitrack" == aTrackerType || "optitrack" == aTrackerType || "optiTrack" == aTrackerType || "OptiTrack" == aTrackerType)
    {
        enumToReturn = VehicleStateTracker::FetcherType::OPTITRACK; 
    }
    else
    {
        LOGE << "Unsupported state fetcher type: " << aTrackerType; 
    }

    return enumToReturn; 
}

void VehicleStateTracker::stateTrackerLoop()
{  
    RateController rate(mConfig.mRate); 
    
    if(!mStateFetcher->init())
    {
        LOGE << "Could not initialize state fetcher of type: " << mConfig.mInterface; 
        return; 
    }
    
    // wait until state is acquired 
    while(!mStateFetcher->isStateAcquired())
    {
        LOGD << "Waiting for state data from " << mStateFetcher->type();
        sleep(1); 
    }

    // init stuff
    auto logId = DataLogger::get().createLog("abv_state_data");
    LOGD << "Starting state tracking thread";
    setStateTracking(true);
    AbvState stateEstimate;
    mLastMeasurementTime = std::chrono::system_clock::now();
    mIsNavValid = true;

    while (doStateTracking())
    {
        rate.start();

        double dt = rate.getDeltaTime();
        auto stateMeasurement = mStateBuffer.consume();

        if (stateMeasurement.has_value())
        {
            // Predict exactly to this measurement time
            mEKF.predict(dt, stateEstimate);
            mEKF.update(stateMeasurement->state, stateEstimate);
            mLastMeasurementTime = stateMeasurement->measurementTime;
            mIsNavValid = true;
        }
        else if (isWithinDeadReckonBound(std::chrono::system_clock::now(), mLastMeasurementTime, mConfig.mMaxDeadReckonDuration))
        {
            // Still within the trusted dead-reckoning window: propagate forward
            mEKF.predict(dt, stateEstimate);
        }
        else
        {
            // Free-running past the bound: stop extrapolating and mark invalid,
            // rather than letting position/velocity diverge indefinitely
            mIsNavValid = false;
        }

        AbvState stateToPublish = stateEstimate;
        if (!mIsNavValid)
        {
            // no basis to trust extrapolated velocity once we've stopped predicting
            stateToPublish.vx = 0.0;
            stateToPublish.vy = 0.0;
            stateToPublish.omega = 0.0;
        }

        mStatePublisher.publish(stateToPublish, mLastMeasurementTime, mIsNavValid);
        DataLogger::get().write(logId, toVector(stateEstimate));

        rate.block();
    }
}

void VehicleStateTracker::thrusterStatusCallback(abv_msgs::msg::AbvThrusterStatus::ConstSharedPtr aMsg)
{
    Eigen::Vector3d appliedThrust;
    appliedThrust << aMsg->fx, aMsg->fy, aMsg->tz;
    mEKF.setLatestInput(appliedThrust);
}

inline bool VehicleStateTracker::isWithinDeadReckonBound(std::chrono::system_clock::time_point aNow,
                                     std::chrono::system_clock::time_point aLastMeasurementTime,
                                     double aBoundDuration_s)
{
    return std::chrono::duration<double>(aNow - aLastMeasurementTime).count() <= aBoundDuration_s;
}
