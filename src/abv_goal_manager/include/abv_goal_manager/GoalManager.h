#ifndef GOALMANAGER_H
#define GOALMANAGER_H

#include <eigen3/Eigen/Dense>
#include <mutex>
#include <memory>

#include "abv_common/ArrivalStatus.hpp"
#include "abv_common/ThreadSafe.hpp"
#include "abv_common/RosNavigationListener.h"
#include "abv_common/Configurations.h"
#include "abv_common/ConfigurationManager.h"

#include "abv_goal_manager/GoalType.hpp"

class GoalManager
{
public:
    GoalManager();
    ~GoalManager();

    void doThrusterControl();
    void doDirectionControl();
    void doPoseControl();
    void doVelocityControl();

    void setGoalPose(Eigen::Vector3d aGoalPose);
    void setGoalVelocity(Eigen::Vector3d aGoalVel, bool anIsGlobal = true);
    void setControlInput(Eigen::Vector3d aControlInput, bool anIsGlobal = true);
    void setThrusterCmdSequence(const std::string& aCmd);

    void setArrivalTolerance(const Eigen::Vector3d& aTolerance);

    void stop();

    // zeroes thrusters via the real control path without touching the
    // current goal, so the goal can be auto-resumed once nav data recovers
    void safeStop();

    // clears the active goal (state machine goes to IDLE) WITHOUT publishing
    // a thruster command, unlike stop() - used on a control-policy switch so
    // the vehicle isn't driven by a goal computed for the policy that just
    // got swapped out, without issuing a new command of its own
    void clearGoal();

    Eigen::Vector3d getGoalPose();
    Eigen::Vector3d getGoalVelocity();
    Eigen::Vector3d getControlInput();

    bool isControlInputStale();
    bool hasAcquiredStateData();

    // true iff nav data is currently arriving and valid (fresh && valid)
    bool isNavOk();

    // true iff the currently active goal actually depends on nav data.
    // POSE_CONTROL/VELOCITY_CONTROL always need it; DIRECTION_CONTROL only
    // needs it for global-frame commands (body-frame never touches nav data).
    bool needsFreshNavData();
    struct ControlStatus
    {
        Arrival::Status mStatus;
    };

    ControlStatus getControlStatus();

private:
    Eigen::Vector3d mGoalPose;
    Eigen::Vector3d mGoalVelocity;
    Eigen::Vector3d mControlInput;

    ControlConfig mConfig;

    ThreadSafe<Eigen::Vector3d> mPoseError;
    ThreadSafe<Eigen::Vector3d> mVelError;
    ThreadSafe<bool> mJustRecvdNewGoal;
    ThreadSafe<bool> mIsGoalGlobal;

    Eigen::Vector3d mArrivalTol;
    std::chrono::steady_clock::time_point mArrivalStart;
    bool mArrivalTimerActive;


    std::chrono::steady_clock::time_point mLastInputRecvdAt;
    std::chrono::duration<double> mStaleInputThreshold;

    GoalType mGoalType;

    std::mutex mGoalPoseMutex;
    std::mutex mControlInputMutex;
    std::mutex mGoalVelocityMutex;
    std::mutex mGoalToleranceMutex;

    std::shared_ptr<RosNavigationListener> mNavManager;

    ThreadSafe<std::string> mThrusterCmdSequence;

private:
    // used only to compute a matching-frame velocity error in
    // doVelocityControl() - NOT for converting the final control input
    // before commanding thrusters, which abv_thrusters now owns.
    Eigen::Vector3d convertToBodyFrame(Eigen::Vector3d aVectorGlobal);

    void publishThrusterCommand(const std::string& aType, Eigen::Vector3d aData, bool anIsGlobal, const std::string& aThrusters = "");
    Arrival::Status determineArrivalStatus();

};
#endif // GOALMANAGER_H
