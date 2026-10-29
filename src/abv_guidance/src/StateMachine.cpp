
#include <cmath>
#include <iostream>
#include "plog/Log.h"

#include "abv_msgs/msg/abv_controller_command.hpp"
#include "abv_msgs/msg/abv_guidance_status.hpp"
#include "abv_msgs/msg/abv_path.hpp"
#include "abv_msgs/msg/dock_command.hpp"

#include "abv_common/RateController.hpp"
#include "abv_common/ConfigurationManager.h"
#include "abv_common/RosTopicManager.h"

#include "abv_guidance/StateMachine.h"
#include "abv_guidance/StraightLineGenerator.h"
#include "abv_guidance/FromFileGenerator.h"
#include "abv_guidance/CollisionAvoidPathGenerator.h"
#include "abv_guidance/CyclicDockGenerator.h"

StateMachine::StateMachine() : 
    mDone(false), mActiveState(States::STARTUP), mApproachCount(0)
{
    mDockLocked.set(false);
    mDockLinkOk.set(false);
    mDockingConfig = ConfigurationManager::getInstance()->getDockingConfig();

    RosTopicManager::getInstance()->createPublisher<abv_msgs::msg::DockCommand>("abv/dock/cmd");
    RosTopicManager::getInstance()->createSubscriber<abv_msgs::msg::DockStatus>("abv/dock/status",
            std::bind(&StateMachine::dockStatusCallback, this, std::placeholders::_1));

    RosTopicManager::getInstance()->createPublisher<abv_msgs::msg::AbvControllerCommand>("abv/controller/command");
    RosTopicManager::getInstance()->createPublisher<abv_msgs::msg::AbvGuidanceStatus>("abv/guidance/status");
    RosTopicManager::getInstance()->createPublisher<abv_msgs::msg::AbvPath>("abv/guidance/path");

    RosTopicManager::getInstance()->createSubscriber<abv_msgs::msg::AbvControllerStatus>("abv/controller/status", 
            std::bind(&StateMachine::controllerStatusCallback, this, std::placeholders::_1)); 
}

StateMachine::~StateMachine()
{
    mWaypointWatchdog.cancel();
    mWatchdog.cancel();

    if(mStatusPublishThread.joinable())
    {
        mStatusPublishThread.join();
    }
}

void StateMachine::onCommand(const Command& aCommand)
{
    mCommand = aCommand;
    LOGV << "Recvd command to execute " << mCommand.mType << " path for " << mCommand.mDuration << " seconds!";
    if(States::STARTUP == getActiveState())
    {
        LOGW << "Cannot execute trajectory, navigation data not yet acquired..."; 
        return; 
    }  
    
    setActiveState(States::GENERATE_PATH); 
}

void StateMachine::run()
{
    int rateConfig = ConfigurationManager::getInstance()->getGuidanceConfig().mStateMachineRate; 
    RateController rate(rateConfig); 

    mStatusPublishThread = std::thread(&StateMachine::statusPublishLoop, this);

    LOGD << "State Machine starting in " << toString(mActiveState);

    while(!isDone())
    {
        rate.start(); 

        switch (getActiveState())
        {
        case States::STARTUP: 
            startup();  
            break;

        case States::IDLE:
            // do nothing 
            break;

        case States::GENERATE_PATH:
            generatePath(); 
            break; 

        case States::SEND_WAYPOINT:
            sendWaypoint(); 
            break;
        
        case States::WAITING_FOR_EXECUTION: 
            waitForExecution(); 
            break; 

        case States::WAITING_FOR_ARRIVAL:
            waitForArrival(); 
            break; 

        default:
            break;
        }

        rate.block(); 
    }
}

void StateMachine::startup()
{
    // wait for nav data, once received transition to IDLE.
    if(mNavSource.hasAcquiredStateData())
    {
        LOGV << "Acquired navigation data!"; 
        setActiveState(States::IDLE);
    }
}

void StateMachine::generatePath()
{
    // get the latest command, instantiate the proper IPathGenerator based on the command
    // and do any startup/init stuff for that IPathGenerator 
    if("line" == mCommand.mType || "Line" == mCommand.mType)
    {
        auto pathGen = std::make_unique<StraightLineGenerator>(mCommand.mGoal);
        mPathGenerator = std::move(pathGen);
    }
    else if("file" == mCommand.mType || "File" == mCommand.mType)
    {
        auto pathGen = std::make_unique<FromFileGenerator>();
        mPathGenerator = std::move(pathGen);
    }
    else if("avoid" == mCommand.mType || "Avoid" == mCommand.mType)
    {
        auto pathGen = std::make_unique<CollisionAvoidPathGenerator>(mCommand.mGoal, mNavSource, mScene);
        mPathGenerator = std::move(pathGen);
    }
    else if("dock" == mCommand.mType || "Dock" == mCommand.mType)
    {
        // goal x = radial offset (m), goal yaw = angular offset (rad); see CyclicDockGenerator.h
        mApproachCount = 0;
        auto pathGen = std::make_unique<CyclicDockGenerator>(mCommand.mGoal.mPose.x(), mCommand.mGoal.mPose.z(), mScene);
        mPathGenerator = std::move(pathGen);
    }
    else
    {
        LOGW << "Unknown path generator of type: " << mCommand.mType; 
        setActiveState(States::IDLE); 
        return; 
    }

    // init the path generator 
    if(!mPathGenerator->init())
    {
        LOGW << "Failed to initialize path generator..."; 
        setActiveState(States::IDLE); 
        return; 
    }

    setActiveState(States::SEND_WAYPOINT); 
    if (mCommand.mDuration != -1)
        mWatchdog.start(mCommand.mDuration, std::bind(&StateMachine::onTimeout, this));  
}

void StateMachine::sendWaypoint()
{
    // snapshot the remaining path *before* consuming the next waypoint, so
    // FromFileGenerator's mIndex-based slice still includes the one about to
    // be dispatched (see IPathGenerator::getPath()) - truncated to what the
    // generator advises is actually meaningful to publish right now.
    std::vector<Waypoint> fullPath = mPathGenerator->getPath();
    std::size_t previewLen = std::min(mPathGenerator->getPathPreviewLength(), fullPath.size());
    mCurrentPath.set(std::vector<Waypoint>(fullPath.begin(), fullPath.begin() + previewLen));

    // get next waypoint from current IPathGenerator and send via ROS2
    Waypoint wp = mPathGenerator->getNext();
    mCurrentWaypoint = wp;

    // Non-pose steps (docking additions). These don't wait on the controller's
    // ARRIVED status; waitForArrival() ends them with isTimedStepComplete().
    if(wp.mType == WaypointType::VELOCITY || wp.mType == WaypointType::APPROACH)
    {
        if(wp.mType == WaypointType::APPROACH)
        {
            mApproachCount++;
            Eigen::Vector3d pose = mNavSource.getCurrentPose();
            Eigen::Vector3d vel = mNavSource.getCurrentVel();
            LOGI << "Approach " << mApproachCount << " starting: commanded "
                 << std::hypot(wp.mVel.x(), wp.mVel.y()) << " m/s, current speed "
                 << std::hypot(vel.x(), vel.y()) << " m/s, face gap "
                 << DockingGeometry::faceGap(mDockingConfig, pose.x(), pose.y(), pose.z())
                 << " m, lateral offset "
                 << DockingGeometry::faceLateralOffset(mDockingConfig, pose.x(), pose.y(), pose.z()) << " m";
            if(mDockLocked.get())
            {
                LOGW << "Mechanism already reports LOCKED at the start of the approach - check that the last unlock worked";
            }
        }
        sendVelocityCommand(wp.mVel);
        mStepStart = std::chrono::steady_clock::now();
        setActiveState(States::WAITING_FOR_ARRIVAL);
        return;
    }
    else if(wp.mType == WaypointType::HOLD)
    {
        // actively hold zero velocity. STOP only switches the thrusters off,
        // which on a frictionless table lets the vehicle keep coasting.
        LOGV << "Holding for " << wp.mTimeout << " s";
        sendVelocityCommand(Eigen::Vector3d::Zero());
        mStepStart = std::chrono::steady_clock::now();
        setActiveState(States::WAITING_FOR_ARRIVAL);
        return;
    }
    else if(wp.mType == WaypointType::DOCK_COMMAND)
    {
        sendDockCommand(wp.mDockCommand);
        finishStep();
        return;
    }
    else if(wp.mType != WaypointType::POSE)
    {
        LOGE << "Unknown waypoint type '" << wp.mType << "', aborting path";
        mWatchdog.cancel();
        sendStopCommand();
        setActiveState(States::IDLE);
        return;
    }

    // convert to idl type and publish
    abv_msgs::msg::AbvControllerCommand cmd;
    cmd.set__type(wp.mType);

    abv_msgs::msg::AbvVec3 vec;
    vec.set__x(wp.mPose.x());
    vec.set__y(wp.mPose.y());
    vec.set__yaw(wp.mPose.z());

    cmd.set__data(vec);

    // set waypoint tolerance: per-waypoint override if this waypoint
    // specified one, else the command-level default
    Eigen::Vector3d resolvedTol = resolveArrivalTolerance(wp);
    abv_msgs::msg::AbvVec3 tol;
    tol.set__x(resolvedTol.x());
    tol.set__y(resolvedTol.y());
    tol.set__yaw(resolvedTol.z());

    cmd.set__tolerance(tol);

    LOGV << "Sending next waypoint...";
    RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvControllerCommand>("abv/controller/command", cmd);
    setActiveState(States::WAITING_FOR_EXECUTION);

    // per-waypoint timeout: falls back to the configured default unless the
    // path generator supplied its own override for this waypoint
    double waypointTimeout = wp.mTimeout > 0.0
        ? wp.mTimeout
        : ConfigurationManager::getInstance()->getGuidanceConfig().mWaypointTimeout;

    mWaypointWatchdog.start(waypointTimeout, std::bind(&StateMachine::onWaypointTimeout, this));
}

void StateMachine::waitForExecution()
{
    if(mArrivalStatus.get() == Arrival::Status::RUNNING)
    {
        LOGV << "Controller heard waypoint..."; 
        setActiveState(States::WAITING_FOR_ARRIVAL); 
    }
}

void StateMachine::waitForArrival()
{
    // timed / docking steps finish on their own conditions, not controller ARRIVED
    if(mCurrentWaypoint.mType != WaypointType::POSE)
    {
        if(isTimedStepComplete())
        {
            if(mCurrentWaypoint.mType != WaypointType::HOLD)
            {
                // brake to zero velocity (STOP alone would let it coast)
                sendVelocityCommand(Eigen::Vector3d::Zero());
            }
            finishStep();
        }
        return;
    }

    // query controller status for arrival state, once arrived, send next waypoint
    if(mArrivalStatus.get() == Arrival::Status::ARRIVED)
    {
        // controller arrived on current waypoint, send next
        LOGV << "Controller arrived...";
        mWaypointWatchdog.cancel();

        if(mPathGenerator->hasNext())
        {
            setActiveState(States::SEND_WAYPOINT);
        }
        else
        {
            // arrived at final waypoint, clear path timeout watchdog and go back to IDLE
            mWatchdog.cancel();
            setActiveState(States::IDLE); 
        }
    }
}

void StateMachine::controllerStatusCallback(abv_msgs::msg::AbvControllerStatus::SharedPtr aStatus)
{
    mArrivalStatus.set((Arrival::Status)aStatus->arrival); 
}

void StateMachine::setActiveState(StateMachine::States aState)
{
    LOGD << "Switching state machine from " << toString(getActiveState()).c_str() 
         << " to " << toString(aState).c_str();

    std::lock_guard<std::mutex> lock(mActiveStateMutex); 
    mActiveState = aState;    
}

void StateMachine::onTimeout()
{
    LOGV << "Commanded execution duration has been reached.";

    mWaypointWatchdog.cancel();
    sendStopCommand();

    setActiveState(StateMachine::States::IDLE);
}

void StateMachine::sendStopCommand()
{
    abv_msgs::msg::AbvControllerCommand cmd;
    cmd.set__type("STOP");
    RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvControllerCommand>("abv/controller/command", cmd);
}

Eigen::Vector3d StateMachine::resolveArrivalTolerance(const Waypoint& aWaypoint) const
{
    Eigen::Vector3d resolved;
    Eigen::Vector3d configDefault = ConfigurationManager::getInstance()->getControlConfig().mPoseArrivalTol;

    for(int i = 0; i < 3; i++)
    {
        if(aWaypoint.mArrivalTol[i] > 0)
        {
            resolved[i] = aWaypoint.mArrivalTol[i];
        }
        else if(mCommand.mArrivalTol[i] > 0)
        {
            resolved[i] = mCommand.mArrivalTol[i];
        }
        else
        {
            resolved[i] = configDefault[i];
        }
    }

    return resolved;
}

void StateMachine::onWaypointTimeout()
{
    // stale timeout firing after we've already moved on (e.g. the overall
    // path timeout or a new command already took us out of this waypoint).
    // Non-pose steps time themselves in isTimedStepComplete().
    if((getActiveState() != States::WAITING_FOR_EXECUTION && getActiveState() != States::WAITING_FOR_ARRIVAL) ||
       mCurrentWaypoint.mType != WaypointType::POSE)
    {
        return;
    }

    LOGW << "Timed out waiting for arrival at the current waypoint...";

    Eigen::Vector3d error = mCurrentWaypoint.mPose - mNavSource.getCurrentPose();
    error.z() = std::atan2(std::sin(error.z()), std::cos(error.z())); // wrap yaw to [-pi, pi]

    // use the tolerance actually resolved (and sent to the controller) for
    // this waypoint, not the raw (possibly unset/sentinel) command tolerance
    Eigen::Vector3d relaxedTol = resolveArrivalTolerance(mCurrentWaypoint) *
        ConfigurationManager::getInstance()->getGuidanceConfig().mWaypointTimeoutToleranceScale;
    bool withinRelaxedTol = (error.array().abs() < relaxedTol.array()).all();

    if(withinRelaxedTol)
    {
        LOGW << "Within relaxed arrival bounds, treating waypoint as reached and continuing...";

        if(mPathGenerator->hasNext())
        {
            setActiveState(States::SEND_WAYPOINT);
        }
        else
        {
            // final waypoint accepted via relaxed tolerance - hold station here,
            // same as a normal verified arrival (see waitForArrival()), and
            // clear the path timeout watchdog so it can't fire a stray STOP later.
            mWatchdog.cancel();
            setActiveState(States::IDLE);
        }
    }
    else
    {
        LOGW << "Not within relaxed arrival bounds, aborting path...";

        mWatchdog.cancel();
        sendStopCommand();
        setActiveState(States::IDLE);
    }
}

std::string StateMachine::toString(StateMachine::States aState)
{
    std::string stringToReturn = ""; 
    switch (aState)
    {
    case StateMachine::States::STARTUP:
        stringToReturn = "STARTUP"; 
        break; 
    case StateMachine::States::IDLE:
        stringToReturn = "IDLE"; 
        break;
    case StateMachine::States::GENERATE_PATH: 
        stringToReturn = "GENERATE_PATH";
        break; 
    case StateMachine::States::SEND_WAYPOINT: 
        stringToReturn = "SEND_WAYPOINT"; 
        break;
    case StateMachine::States::WAITING_FOR_EXECUTION: 
        stringToReturn = "WAITING_FOR_EXECUTION"; 
        break; 
    case StateMachine::States::WAITING_FOR_ARRIVAL: 
        stringToReturn = "WAITING_FOR_ARRIVAL"; 
        break; 
    default:
        stringToReturn = "UNKNOWN"; 
        break;
    }

    return stringToReturn; 
}

void StateMachine::statusPublishLoop()
{
    // publish control status at same rate as StateMachine loop 
    int rateConfig = ConfigurationManager::getInstance()->getGuidanceConfig().mStateMachineRate; 
    RateController rate(rateConfig); 

    abv_msgs::msg::AbvGuidanceStatus statusToSend;
    abv_msgs::msg::AbvNodeStatus nodeStatus; 
    nodeStatus.set__node_name("guidance");
 
    LOGV << "Starting status publish loop"; 

    while(!isDone())
    {
        rate.start(); 

        nodeStatus.set__node_state(toString(getActiveState())); 
        statusToSend.set__status(nodeStatus); 

        RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvGuidanceStatus>(
                "abv/guidance/status", statusToSend);

        // no active path outside STARTUP/IDLE - publish empty rather than a
        // stale one so the GUI's drawn path clears on arrival/stop
        bool hasActivePath = getActiveState() != States::STARTUP && getActiveState() != States::IDLE;
        std::vector<Waypoint> pathToSend = hasActivePath ? mCurrentPath.get() : std::vector<Waypoint>{};

        abv_msgs::msg::AbvPath pathMsg;
        for(const Waypoint& wp : pathToSend)
        {
            // hold / velocity / mechanism steps have no position to draw
            if(!wp.hasDisplayPose())
            {
                continue;
            }
            abv_msgs::msg::AbvVec3 pt;
            pt.set__x(wp.mPose.x());
            pt.set__y(wp.mPose.y());
            pt.set__yaw(wp.mPose.z());
            pathMsg.waypoints.push_back(pt);
        }

        RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvPath>(
                "abv/guidance/path", pathMsg);

        rate.block();
    }
}

void StateMachine::sendVelocityCommand(const Eigen::Vector3d& aVel)
{
    abv_msgs::msg::AbvControllerCommand cmd;
    cmd.set__type(kControllerVelocityType);
    cmd.set__is_global(true); // table frame, so the approach stays on the docking axis whatever the heading

    abv_msgs::msg::AbvVec3 vec;
    vec.set__x(aVel.x());
    vec.set__y(aVel.y());
    vec.set__yaw(aVel.z());
    cmd.set__data(vec);

    RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvControllerCommand>("abv/controller/command", cmd);
}

void StateMachine::sendDockCommand(std::uint8_t aCode)
{
    abv_msgs::msg::DockCommand cmd;
    cmd.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    cmd.command = aCode;
    LOGI << "Dock command " << static_cast<int>(aCode)
         << " (1=ARM 2=DISARM 3=LOCK 4=UNLOCK), link " << (mDockLinkOk.get() ? "OK" : "DOWN");
    RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::DockCommand>("abv/dock/cmd", cmd);
}

double StateMachine::stepElapsed_s() const
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - mStepStart).count();
}

bool StateMachine::isTimedStepComplete()
{
    const double elapsed = stepElapsed_s();
    const double limit = mCurrentWaypoint.mTimeout > 0.0
        ? mCurrentWaypoint.mTimeout
        : ConfigurationManager::getInstance()->getGuidanceConfig().mWaypointTimeout;

    if(mCurrentWaypoint.mType == WaypointType::APPROACH)
    {
        Eigen::Vector3d pose = mNavSource.getCurrentPose();
        Eigen::Vector3d vel = mNavSource.getCurrentVel();
        double gap = DockingGeometry::faceGap(mDockingConfig, pose.x(), pose.y(), pose.z());

        if(mDockLocked.get() && mDockLinkOk.get())
        {
            LOGI << "Approach " << mApproachCount << " result: CAPTURED after " << elapsed
                 << " s, face gap " << gap << " m, speed " << std::hypot(vel.x(), vel.y()) << " m/s";
            return true;
        }
        if(gap <= mDockingConfig.mCaptureRange)
        {
            LOGW << "Approach " << mApproachCount << " result: NO LOCK at capture range after " << elapsed
                 << " s, face gap " << gap << " m";
            return true;
        }
        if(elapsed >= limit)
        {
            LOGW << "Approach " << mApproachCount << " result: TIMED OUT after " << elapsed
                 << " s, face gap " << gap << " m";
            return true;
        }
        return false;
    }

    // VELOCITY and HOLD just run for their duration
    return elapsed >= limit;
}

void StateMachine::finishStep()
{
    if(mPathGenerator->hasNext())
    {
        setActiveState(States::SEND_WAYPOINT);
    }
    else
    {
        mWatchdog.cancel();
        setActiveState(States::IDLE);
    }
}

void StateMachine::dockStatusCallback(abv_msgs::msg::DockStatus::SharedPtr aStatus)
{
    mDockLocked.set(aStatus->locked);
    mDockLinkOk.set(aStatus->link_ok);
}
