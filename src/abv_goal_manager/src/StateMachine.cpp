
#include "abv_goal_manager/StateMachine.h"
#include "abv_common/RateController.hpp"
#include "abv_common/ConfigurationManager.h"
#include "abv_common/RosTopicManager.h"
#include <iostream> 
#include "plog/Log.h"

#include "abv_msgs/msg/abv_controller_status.hpp"

StateMachine::StateMachine(std::shared_ptr<GoalManager> abv) :
    mDone(false), mActiveState(States::STARTUP), mGoalManager(abv)
{
    RosTopicManager::getInstance()->createPublisher<abv_msgs::msg::AbvControllerStatus>("abv/controller/status");
}

StateMachine::~StateMachine()
{
    if(mControlStatusPublishThread.joinable())
    {
        mControlStatusPublishThread.join(); 
    }
}

void StateMachine::run()
{
    int rateConfig = ConfigurationManager::getInstance()->getControlConfig().mStateMachineRate; 
    RateController rate(rateConfig); 

    mControlStatusPublishThread = std::thread(&StateMachine::controlStatusPublishLoop, this);

    LOGD << "State Machine starting in " << toString(mActiveState);

    while(!isDone())
    {
        rate.start(); 

        switch (getActiveState())
        {
        case States::IDLE:
            // do nothing 
            break;

        case States::THRUSTER_CONTROL: 
            mGoalManager->doThrusterControl(); 
            break; 
        case States::DIRECTION_CONTROL:

            if(mGoalManager->isControlInputStale())
            {
                mGoalManager->stop();
                setActiveState(States::IDLE);
                break;
            }

            if(mGoalManager->needsFreshNavData() && !mGoalManager->isNavOk())
            {
                // global-frame direction command lost nav data; explicitly
                // reset mPreFaultState to IDLE (rather than leaving it
                // untouched) so recovery can't replay a stale POSE_CONTROL/
                // VELOCITY_CONTROL goal left over from an earlier fault
                mPreFaultState = States::IDLE;
                mGoalManager->safeStop();
                setActiveState(States::NAV_FAULT);
                break;
            }

            // if here, control input not stale, apply it
            mGoalManager->doDirectionControl();
            break;

        case States::POSE_CONTROL:

            if(!mGoalManager->isNavOk())
            {
                mPreFaultState = States::POSE_CONTROL;
                mGoalManager->safeStop();
                setActiveState(States::NAV_FAULT);
                break;
            }

            mGoalManager->doPoseControl();
            break;

        case States::VELOCITY_CONTROL:

            if(!mGoalManager->isNavOk())
            {
                mPreFaultState = States::VELOCITY_CONTROL;
                mGoalManager->safeStop();
                setActiveState(States::NAV_FAULT);
                break;
            }

            mGoalManager->doVelocityControl();
            break;

        case States::NAV_FAULT:

            // re-issue every tick while faulted as a belt-and-suspenders retry
            mGoalManager->safeStop();

            if(mGoalManager->isNavOk())
            {
                setActiveState(mPreFaultState);
            }
            break;

        default:
            break;
        }

        rate.block(); 
    }
}

void StateMachine::setActiveState(StateMachine::States aState)
{
    LOGD << "Switching state machine from " << toString(getActiveState()).c_str() << " to " 
                                                    << toString(aState).c_str();

    std::lock_guard<std::mutex> lock(mActiveStateMutex); 
    mActiveState = aState;    
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
    case StateMachine::States::THRUSTER_CONTROL: 
        stringToReturn = "THRUSTER_CONTROL";
        break; 
    case StateMachine::States::POSE_CONTROL: 
        stringToReturn = "POSE_CONTROL"; 
        break;
    case StateMachine::States::VELOCITY_CONTROL: 
        stringToReturn = "VELOCITY_CONTROL"; 
        break; 
    case StateMachine::States::DIRECTION_CONTROL:
        stringToReturn = "DIRECTION_CONTROL";
        break;
    case StateMachine::States::NAV_FAULT:
        stringToReturn = "NAV_FAULT";
        break;
    default:
        stringToReturn = "UNKNOWN"; 
        break;
    }

    return stringToReturn; 
}

void StateMachine::controlStatusPublishLoop()
{
    // publish control status at same rate as StateMachine loop 
    int rateConfig = ConfigurationManager::getInstance()->getControlConfig().mStateMachineRate; 
    RateController rate(rateConfig); 

    abv_msgs::msg::AbvControllerStatus statusToSend;
    abv_msgs::msg::AbvNodeStatus nodeStatus;
    nodeStatus.set__node_name("goal_manager");

    LOGV << "Starting control status publish loop";

    while(!isDone())
    {
        rate.start();

        nodeStatus.set__node_state(toString(getActiveState()));
        statusToSend.set__status(nodeStatus);

        GoalManager::ControlStatus status = mGoalManager->getControlStatus();
        statusToSend.set__arrival((uint8_t)status.mStatus);
        statusToSend.set__nav_ok(mGoalManager->isNavOk());

        RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvControllerStatus>("abv/controller/status", statusToSend);

        rate.block();
    }
}