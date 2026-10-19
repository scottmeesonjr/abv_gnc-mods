#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <memory>
#include <thread>
#include <mutex>

#include "abv_msgs/msg/abv_controller_status.hpp"

#include "abv_common/ArrivalStatus.hpp"
#include "abv_common/RosNavigationListener.h"
#include "abv_common/ThreadSafe.hpp"

#include "abv_guidance/IPathGenerator.hpp"
#include "abv_guidance/InternalTypes.hpp"
#include "abv_guidance/ICommandSink.hpp"
#include "abv_guidance/PathWatchdog.h"
#include "abv_guidance/Scene.h"

class StateMachine : public ICommandSink
{
public:
    StateMachine();
    ~StateMachine();

    enum class States
    {
        STARTUP,
        IDLE, 
        GENERATE_PATH, 
        SEND_WAYPOINT, 
        WAITING_FOR_EXECUTION,
        WAITING_FOR_ARRIVAL, 
        NUM_TYPES
    };

    void run(); 

    std::string toString(States aState); 

    /** Getters and Setters **/
    bool isDone() {std::lock_guard<std::mutex> lock(mDoneMutex); return mDone; }
    void stop() {std::lock_guard<std::mutex> lock(mDoneMutex); mDone = true; }
    void setActiveState(States aNewActiveState); 
    States getActiveState() {std::lock_guard<std::mutex> lock(mActiveStateMutex); return mActiveState;}

private:
    bool mDone; 
    std::mutex mDoneMutex; 
    States mActiveState; 
    std::mutex mActiveStateMutex; 

    Command mCommand;

    std::unique_ptr<IPathGenerator> mPathGenerator;
    RosNavigationListener mNavSource;
    PathWatchdog mWatchdog;
    PathWatchdog mWaypointWatchdog;
    Waypoint mCurrentWaypoint;

    // Table bounds + live obstacle set. Not yet consulted by generatePath() -
    // this is the hook point for a future CollisionAvoidPathGenerator.
    Scene mScene;

    // Remaining path (from the in-flight waypoint onward) of the active
    // IPathGenerator, refreshed each time a waypoint is dispatched (see
    // sendWaypoint()) and published at a constant rate by statusPublishLoop()
    // for GUI visualization.
    ThreadSafe<std::vector<Waypoint>> mCurrentPath;

    ThreadSafe<Arrival::Status> mArrivalStatus;

    std::thread mStatusPublishThread; 

private: 
    void startup(); 
    void generatePath(); 
    void sendWaypoint(); 
    void waitForExecution();
    void waitForArrival();  

    void onCommand(const Command& aCommand) override;
    void onTimeout();
    void onWaypointTimeout();
    void sendStopCommand();

    // per-axis: aWaypoint's override if > 0, else mCommand.mArrivalTol
    // (which itself falls back to the configured default in
    // GoalManager::setArrivalTolerance)
    Eigen::Vector3d resolveArrivalTolerance(const Waypoint& aWaypoint) const;
    void controllerStatusCallback(abv_msgs::msg::AbvControllerStatus::SharedPtr aStatus);

    void statusPublishLoop();

};
#endif // STATEMACHINE_H
