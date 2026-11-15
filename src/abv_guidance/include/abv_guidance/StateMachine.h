#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <mutex>

#include "abv_msgs/msg/abv_controller_status.hpp"
#include "abv_msgs/msg/dock_status.hpp"

#include "abv_common/ArrivalStatus.hpp"
#include "abv_common/Configurations.h"
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

    // Docking: latest mechanism status from abv/dock/status (abv_dock_bridge)
    ThreadSafe<bool> mDockLocked;
    ThreadSafe<bool> mDockLinkOk;
    DockingConfig mDockingConfig;

    // start time of the current timed step (vel / approach / hold waypoints)
    std::chrono::steady_clock::time_point mStepStart;
    int mApproachCount;

    // approach: set once the face gap first drops inside Docking.CaptureRange
    bool mReachedCaptureRange = false;
    std::chrono::steady_clock::time_point mCaptureRangeReachedAt;

    // approach: face lateral offset when the approach started. Held for the
    // whole approach so the commanded test offset is what the mechanism sees
    // at contact (it is preserved, never corrected toward the axis).
    double mApproachLateral0 = 0.0;

    // rad, yaw the approach holds: aligned + the dock run's commanded angular
    // offset. Without this, any yaw rate left over from the standoff move
    // drifts the heading during the approach (velocity mode only damps rate).
    double mApproachYawTarget = 0.0;
    double mDockAngularOffset = 0.0;

    // dock run bookkeeping for the abv/dock/event messages (GUI pop-up)
    bool mDockRunActive = false;
    int mCapturedCount = 0;
    int mDockTotalCycles = 0;

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

    // Docking / timed-step support (non-"pose" waypoint types, see InternalTypes.hpp)
    void sendVelocityCommand(const Eigen::Vector3d& aVel);
    void sendDockCommand(std::uint8_t aCode);
    bool isTimedStepComplete();
    void finishStep();
    double stepElapsed_s() const;
    void dockStatusCallback(abv_msgs::msg::DockStatus::SharedPtr aStatus);
    void holdApproachLateral();
    // abv/dock/event (std_msgs/String), '|' separated:
    //   CYCLE|<n>|<CAPTURED / NO LOCK / TIMED OUT / OVERTRAVEL>|<speed m/s>
    //   COMPLETE|<captured>|<attempted>
    //   ABORTED|<reason>
    //   STEP|<cycle>|<total cycles>|<step number, 0 = end of run>|<label>
    //        labels: STANDOFF / BACK OFF (1), SETTLE (2), ARM (3), APPROACH (4),
    //                LOCK (5), UNLOCK (6), RELEASE (7); final BACK OFF / DISARM (0)
    void publishDockEvent(const std::string& aEvent);
    void endDockRun(const std::string& aAbortReason);

    // per-axis: aWaypoint's override if > 0, else mCommand.mArrivalTol
    // (which itself falls back to the configured default in
    // GoalManager::setArrivalTolerance)
    Eigen::Vector3d resolveArrivalTolerance(const Waypoint& aWaypoint) const;
    void controllerStatusCallback(abv_msgs::msg::AbvControllerStatus::SharedPtr aStatus);

    void statusPublishLoop();

};
#endif // STATEMACHINE_H
