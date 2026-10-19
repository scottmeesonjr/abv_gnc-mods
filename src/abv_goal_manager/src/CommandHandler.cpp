
#include "abv_goal_manager/CommandHandler.h"
#include "plog/Log.h"


CommandHandler::CommandHandler(std::shared_ptr<StateMachine> msm, std::shared_ptr<GoalManager> abv) : 
    mStateMachine(msm), mGoalManager(abv)
{
    auto topicManager = RosTopicManager::getInstance(); 
    topicManager->createSubscriber<abv_msgs::msg::AbvControllerCommand>("abv/controller/command", 
                                    std::bind(&CommandHandler::commandCallback, this, std::placeholders::_1)); 

    topicManager->spinNode(); 

    while (!topicManager->isROSInitialized())
    {
    }
    
    LOGD << "ROS Comms Initialized";
}

CommandHandler::~CommandHandler()
{
    
}

void CommandHandler::commandCallback(abv_msgs::msg::AbvControllerCommand::SharedPtr aCmdMsg)
{   
    auto command = toEnum(aCmdMsg->type); 
    if(CommandType::NUM_TYPES == command)
    {
        LOGW << "Cannot execute unknown command"; 
        return; 
    }

    switch (command)
    {
        case CommandType::STOP:
        {
            mGoalManager->stop();
            setNewActiveState(StateMachine::States::IDLE);

            break;
        }
        case CommandType::CLEAR:
        {
            // unlike STOP, this does not publish a new thruster command -
            // used by abv_policy_registry after a successful policy switch
            // to clear the active goal without driving the vehicle at all
            mGoalManager->clearGoal();
            setNewActiveState(StateMachine::States::IDLE);

            break;
        }
        case CommandType::THRUSTER:
        {
            mGoalManager->setThrusterCmdSequence(aCmdMsg->thrusters); 
            setNewActiveState(StateMachine::States::THRUSTER_CONTROL); 
            break; 
        }
        case CommandType::DIRECTION:
        {
            // body-frame direction commands never touch nav data (see
            // GoalManager::doDirectionControl); global-frame ones need current
            // yaw to rotate into body frame, so they require nav data.
            if(aCmdMsg->is_global && !mGoalManager->isNavOk())
            {
                LOGW << "Cannot execute global-frame direction command, navigation data not fresh/valid...";
                return;
            }

            mGoalManager->setControlInput(convertToEigen(aCmdMsg->data), aCmdMsg->is_global);
            setNewActiveState(StateMachine::States::DIRECTION_CONTROL);
            break;
        }
        case CommandType::POSE:
        {
            if(!mGoalManager->isNavOk())
            {
                LOGW << "Cannot execute pose command, navigation data not fresh/valid...";
                return;
            }

            mGoalManager->setGoalPose(convertToEigen(aCmdMsg->data));
            mGoalManager->setArrivalTolerance(convertToEigen(aCmdMsg->tolerance));
            setNewActiveState(StateMachine::States::POSE_CONTROL);
            break;
        }
        case CommandType::VELOCITY:
        {
            // gated unconditionally regardless of frame: even the body-frame
            // path in GoalManager::doVelocityControl still reads current pose and
            // rotates current velocity by nav-derived yaw, so there's no
            // legitimate nav-free velocity control path today.
            if(!mGoalManager->isNavOk())
            {
                LOGW << "Cannot execute velocity command, navigation data not fresh/valid...";
                return;
            }

            mGoalManager->setGoalVelocity(convertToEigen(aCmdMsg->data), aCmdMsg->is_global);
            mGoalManager->setArrivalTolerance(convertToEigen(aCmdMsg->tolerance));
            setNewActiveState(StateMachine::States::VELOCITY_CONTROL);
            break;
        }
        default: 
        {
            LOGW << "Cannot execute unknown command"; 
            return; 
        }
    }
}

Eigen::Vector3d CommandHandler::convertToEigen(abv_msgs::msg::AbvVec3 aVectorToConvert)
{
    Eigen::Vector3d vectorToReturn;
    vectorToReturn[0] = aVectorToConvert.x; 
    vectorToReturn[1] = aVectorToConvert.y; 
    vectorToReturn[2] = aVectorToConvert.yaw; 

    return vectorToReturn; 
}

void CommandHandler::setNewActiveState(StateMachine::States aNewState)
{
    auto currentState = mStateMachine->getActiveState(); 

    if(aNewState != currentState)
    { 
        mStateMachine->setActiveState(aNewState); 
    }
}

CommandHandler::CommandType CommandHandler::toEnum(const std::string& aType)
{
    CommandType enumToReturn; 

    if("Thruster" == aType || "thruster" == aType)
    {
        enumToReturn = CommandType::THRUSTER; 
    }
    else if ("Direction" == aType || "direction" == aType)
    {
        enumToReturn = CommandType::DIRECTION; 
    }
    else if ("Pose" == aType || "pose" == aType)
    {
        enumToReturn = CommandType::POSE; 
    }
    else if ("Velocity" == aType || "vel" == aType || "velocity" == aType)
    {
        enumToReturn = CommandType::VELOCITY; 
    }
    else if ("STOP" == aType || "stop" == aType || "Stop" == aType)
    {
        enumToReturn = CommandType::STOP;
    }
    else if ("CLEAR" == aType || "clear" == aType || "Clear" == aType)
    {
        enumToReturn = CommandType::CLEAR;
    }
    else
    {
        LOGW << "Unsupported CommandType: " << aType; 
        enumToReturn = CommandType::NUM_TYPES; 
    }

    return enumToReturn; 
}

std::string CommandHandler::toString(CommandType aCmdType)
{
    std::string stringToReturn = ""; 
    switch (aCmdType)
    {
    case CommandType::THRUSTER: 
        stringToReturn = "THRUSTER_CONTROL";
        break; 
    case CommandType::POSE: 
        stringToReturn = "POSE_CONTROL"; 
        break;
    case CommandType::VELOCITY: 
        stringToReturn = "VELOCITY_CONTROL"; 
        break;
    case CommandType::STOP:
        stringToReturn = "STOP";
        break;
    case CommandType::CLEAR:
        stringToReturn = "CLEAR";
        break;
    case CommandType::DIRECTION:
        stringToReturn = "DIRECTION";
    default:
        break;
    }

    return stringToReturn; 
}