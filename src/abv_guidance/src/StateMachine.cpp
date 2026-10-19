
#include <cmath>
#include <iostream>
#include "plog/Log.h"

#include "abv_msgs/msg/abv_controller_command.hpp"
#include "abv_msgs/msg/abv_guidance_status.hpp"
#include "abv_msgs/msg/abv_path.hpp"

#include "abv_common/RateController.hpp"
#include "abv_common/ConfigurationManager.h"
#include "abv_common/RosTopicManager.h"

#include "abv_guidance/StateMachine.h"
#include "abv_guidance/StraightLineGenerator.h"
#include "abv_guidance/FromFileGenerator.h"
#include "abv_guidance/CollisionAvoidPathGenerator.h"

StateMachine::StateMachine() : 
    mDone(false), mActiveState(States::STARTUP)
{
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
    // path timeout or a new command already took us out of this waypoint)
    if(getActiveState() != States::WAITING_FOR_EXECUTION && getActiveState() != States::WAITING_FOR_ARRIVAL)
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