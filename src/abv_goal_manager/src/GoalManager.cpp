
#include <thread>

#include "abv_goal_manager/GoalManager.h"
#include "plog/Log.h"
#include "abv_common/RateController.hpp"
#include "abv_common/RosTopicManager.h"

#include "abv_msgs/msg/abv_control_context.hpp"
#include "abv_msgs/msg/abv_thruster_command.hpp"

namespace
{
    abv_msgs::msg::AbvVec3 toMsg(const Eigen::Vector3d& aVec)
    {
        abv_msgs::msg::AbvVec3 msg;
        msg.x = aVec.x();
        msg.y = aVec.y();
        msg.yaw = aVec.z();
        return msg;
    }
}

GoalManager::GoalManager() :
    mNavManager(std::make_shared<RosNavigationListener>()),
    mLastInputRecvdAt(std::chrono::steady_clock::now()), mStaleInputThreshold(std::chrono::duration<double>(std::chrono::milliseconds(250))),
    mPoseError(), mVelError(),
    mArrivalTimerActive(false),
    mConfig(ConfigurationManager::getInstance()->getControlConfig()),
    mArrivalTol(mConfig.mPoseArrivalTol),
    mGoalType(GoalType::NUM_TYPES)
{
    auto topicManager = RosTopicManager::getInstance();
    topicManager->createPublisher<abv_msgs::msg::AbvControlContext>("abv/controller/context");
    topicManager->createPublisher<abv_msgs::msg::AbvThrusterCommand>("abv/thrusters/command");
}

GoalManager::~GoalManager()
{
}

void GoalManager::doThrusterControl()
{
    // bypass control-law computation and directly command a sequence of thrusters
    publishThrusterCommand("thruster", Eigen::Vector3d::Zero(), false, mThrusterCmdSequence.get());
}

void GoalManager::doDirectionControl()
{
    Eigen::Vector3d controlInput = getControlInput();

    // frame conversion (if is_global) now happens in abv_thrusters, since it
    // must serve any of abv_goal_manager/abv_control/abv_rl-cpp uniformly
    publishThrusterCommand("direction", controlInput, mIsGoalGlobal.get());
}

void GoalManager::doPoseControl()
{
    abv_msgs::msg::AbvControlContext ctx;

    Eigen::Vector3d currentPose = mNavManager->getCurrentPose();
    Eigen::Vector3d currentVelocity = mNavManager->getCurrentVel();
    Eigen::Vector3d goal = getGoalPose();
    Eigen::Vector3d error = goal - currentPose;

    // wrap yaw error to [-pi, pi]
    error[2] = std::atan2(std::sin(error[2]), std::cos(error[2]));
    mPoseError.set(error);

    ctx.pose = toMsg(currentPose);
    ctx.velocity = toMsg(currentVelocity);
    ctx.goal = toMsg(goal);
    ctx.error = toMsg(error);
    ctx.goal_type = abv_msgs::msg::AbvControlContext::POSE;

    RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvControlContext>("abv/controller/context", ctx);
}

void GoalManager::doVelocityControl()
{
    abv_msgs::msg::AbvControlContext ctx;

    Eigen::Vector3d currentPose = mNavManager->getCurrentPose();
    Eigen::Vector3d goal = getGoalVelocity();
    Eigen::Vector3d currentVelocity;

    if(!mIsGoalGlobal.get())
    {
        // if goal is in body frame, convert current velocity to body frame before computing error
        currentVelocity = convertToBodyFrame(mNavManager->getCurrentVel());
    }
    else
    {
        currentVelocity = mNavManager->getCurrentVel();
    }

    Eigen::Vector3d error = goal - currentVelocity;
    mVelError.set(error);

    ctx.pose = toMsg(currentPose);
    ctx.velocity = toMsg(currentVelocity);
    ctx.goal = toMsg(goal);
    ctx.error = toMsg(error);
    ctx.goal_type = abv_msgs::msg::AbvControlContext::VELOCITY;

    RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvControlContext>("abv/controller/context", ctx);
}

void GoalManager::setControlInput(Eigen::Vector3d aControlInput, bool anIsGlobal)
{
    // pls dont banish me for using a single mutex on two resources
    std::lock_guard<std::mutex> lock(mControlInputMutex);
    mControlInput = aControlInput;
    mLastInputRecvdAt = std::chrono::steady_clock::now();
    mIsGoalGlobal.set(anIsGlobal);
}

void GoalManager::setGoalPose(Eigen::Vector3d aGoalPose)
{
    LOGV << "Received goal pose: " << aGoalPose;
    std::lock_guard<std::mutex> lock(mGoalPoseMutex);
    mGoalPose = aGoalPose;
    mGoalType = GoalType::POSE;
    mJustRecvdNewGoal.set(true);
    mIsGoalGlobal.set(true); // pose goals are always in global frame
}

void GoalManager::setGoalVelocity(Eigen::Vector3d aGoalVel, bool anIsGlobal)
{
    LOGV << "Received goal vel: " << aGoalVel;
    std::lock_guard<std::mutex> lock(mGoalVelocityMutex);
    mGoalVelocity = aGoalVel;
    mGoalType = GoalType::VELOCITY;
    mJustRecvdNewGoal.set(true);
    mIsGoalGlobal.set(anIsGlobal);
}

void GoalManager::setArrivalTolerance(const Eigen::Vector3d& aTolerance)
{
    std::lock_guard<std::mutex> lock(mGoalToleranceMutex);

    for(int i = 0; i < 3; i++)
    {
        // if any field is 0 or negative, use the default configured tolerances,
        // otherwise use the value that was commanded
        if(0 >= aTolerance[i])
        {
            mArrivalTol[i] = mConfig.mPoseArrivalTol[i];
        }
        else
        {
            mArrivalTol[i] = aTolerance[i];
        }
    }

    LOGV << "Using waypoint tolerance of " << mArrivalTol;
}

void GoalManager::setThrusterCmdSequence(const std::string& aCmd)
{
    LOGV << "Received thruster command sequence " << aCmd;
    mThrusterCmdSequence.set(aCmd);
}

Eigen::Vector3d GoalManager::getGoalPose()
{
    std::lock_guard<std::mutex> lock(mGoalPoseMutex);
    return mGoalPose;
}

Eigen::Vector3d GoalManager::getGoalVelocity()
{
    std::lock_guard<std::mutex> lock(mGoalVelocityMutex);
    return mGoalVelocity;
}

Eigen::Vector3d GoalManager::getControlInput()
{
    std::lock_guard<std::mutex> lock(mControlInputMutex);
    return mControlInput;
}

bool GoalManager::isControlInputStale()
{
    return std::chrono::steady_clock::now() - mLastInputRecvdAt > mStaleInputThreshold ? true : false;
}

Eigen::Vector3d GoalManager::convertToBodyFrame(Eigen::Vector3d aVectorGlobal)
{
    auto state = mNavManager->getCurrentState();

    // yaw is element 2 TODO: make this more explicit
    double yaw = state[2];

    // invert rotation relative to global
    Eigen::Matrix3d Rz;
    Rz << cos(yaw), sin(yaw), 0,
         -sin(yaw), cos(yaw), 0,
             0,        0,     1;

    // Transform the vector into the new frame
    return Rz * aVectorGlobal;
}

bool GoalManager::hasAcquiredStateData()
{
    return mNavManager->hasAcquiredStateData();
}

bool GoalManager::isNavOk()
{
    return mNavManager->isNavOk();
}

bool GoalManager::needsFreshNavData()
{
    // Only DIRECTION_CONTROL's need is conditional on frame (body-frame
    // direction commands never touch nav data, see doDirectionControl()).
    // POSE_CONTROL/VELOCITY_CONTROL always need nav and are checked via
    // isNavOk() directly, without going through this.
    return mIsGoalGlobal.get();
}

void GoalManager::stop()
{
    LOGV << "Received STOP command!";
    Eigen::Vector3d zeros = Eigen::Vector3d::Zero();
    setControlInput(zeros);
    publishThrusterCommand("direction", zeros, false);
    mGoalType = GoalType::NUM_TYPES; // so arrival state goes to IDLE
}

void GoalManager::safeStop()
{
    LOGW << "Nav data lost - safe stopping thrusters";
    publishThrusterCommand("direction", Eigen::Vector3d::Zero(), false);
    // deliberately leaves mGoalPose/mGoalVelocity/mGoalType untouched so the
    // active goal can be auto-resumed once nav data is fresh again
}

void GoalManager::clearGoal()
{
    LOGV << "Clearing active goal (no new thruster command published)";
    mGoalType = GoalType::NUM_TYPES; // so arrival state goes to IDLE
}

void GoalManager::publishThrusterCommand(const std::string& aType, Eigen::Vector3d aData, bool anIsGlobal, const std::string& aThrusters)
{
    abv_msgs::msg::AbvThrusterCommand cmd;
    cmd.type = aType;
    cmd.is_global = anIsGlobal;
    cmd.data = toMsg(aData);
    cmd.thrusters = aThrusters;

    RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvThrusterCommand>("abv/thrusters/command", cmd);
}

GoalManager::ControlStatus GoalManager::getControlStatus()
{
    ControlStatus cs;
    cs.mStatus = determineArrivalStatus();
    return cs;
}

Arrival::Status GoalManager::determineArrivalStatus()
{
    using clock = std::chrono::steady_clock;

    Arrival::Status status;
    status = Arrival::Status::ARRIVED;

    bool within_threshold = false;

    switch (mGoalType)
    {
    case GoalType::POSE:
        {
            auto error = mPoseError.get();
            within_threshold = (abs(error.array()) < mArrivalTol.array()).all();
            break;
        }
    case GoalType::VELOCITY:
        {
            auto error = mVelError.get();
            within_threshold = (abs(error.array()) < mArrivalTol.array()).all();
            break;
        }
    case GoalType::THRUSTER:
        {
            status = Arrival::Status::RUNNING;
            return status;
        }
    default:
        status = Arrival::Status::IDLE;
        return status;
    }

    // --- Arrival timing logic ---
    const auto now = clock::now();

    if (!within_threshold || mJustRecvdNewGoal.get())
    {
        // Reset timer
        mArrivalTimerActive = false;
        mJustRecvdNewGoal.set(false);
        status = Arrival::Status::RUNNING;
        return status;
    }

    // We are within threshold
    if (!mArrivalTimerActive)
    {
        // Start the timer
        mArrivalTimerActive = true;
        mArrivalStart = now;
        status = Arrival::Status::RUNNING;
        return status;
    }

    // Timer is active — check if enough time has passed
    if (now - mArrivalStart >= std::chrono::seconds((int)mConfig.mArrivalDuration))
    {
        status = Arrival::Status::ARRIVED;
    }
    else
    {
        status = Arrival::Status::RUNNING;
    }

    return status;
}
