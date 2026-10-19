#include "abv_thrusters/ThrusterCommandHandler.h"
#include "abv_common/RosTopicManager.h"
#include "abv_msgs/msg/abv_thruster_status.hpp"
#include "plog/Log.h"

ThrusterCommandHandler::ThrusterCommandHandler() :
    mThrusterCommander(std::make_shared<ThrusterCommander>()),
    mNavManager(std::make_shared<RosNavigationListener>())
{
    auto topicManager = RosTopicManager::getInstance();
    topicManager->createPublisher<abv_msgs::msg::AbvThrusterStatus>("abv/thrusters/status");
    topicManager->createSubscriber<abv_msgs::msg::AbvThrusterCommand>("abv/thrusters/command",
                                    std::bind(&ThrusterCommandHandler::commandCallback, this, std::placeholders::_1));
    topicManager->createSubscriber<abv_msgs::msg::AbvPolicyActive>("abv/policy/active",
                                    std::bind(&ThrusterCommandHandler::policyActiveCallback, this, std::placeholders::_1));

    topicManager->spinNode();
}

ThrusterCommandHandler::~ThrusterCommandHandler()
{
}

void ThrusterCommandHandler::policyActiveCallback(abv_msgs::msg::AbvPolicyActive::SharedPtr aMsg)
{
    mActivePolicy = aMsg->policy_name;
}

void ThrusterCommandHandler::commandCallback(abv_msgs::msg::AbvThrusterCommand::SharedPtr aCmdMsg)
{
    if(!aCmdMsg->policy_name.empty() && aCmdMsg->policy_name != mActivePolicy)
    {
        LOGV << "Dropping thruster command from inactive policy '" << aCmdMsg->policy_name << "' (active: '" << mActivePolicy << "')";
        return;
    }

    if("thruster" == aCmdMsg->type)
    {
        mThrusterCommander->commandThrusters(aCmdMsg->thrusters);
        publishStatus();
        return;
    }

    if("direction" != aCmdMsg->type)
    {
        LOGW << "Unsupported AbvThrusterCommand type: " << aCmdMsg->type;
        return;
    }

    Eigen::Vector3d controlInput(aCmdMsg->data.x, aCmdMsg->data.y, aCmdMsg->data.yaw);

    if(aCmdMsg->is_global)
    {
        controlInput = convertToBodyFrame(controlInput);
    }

    mThrusterCommander->command(controlInput);
    publishStatus();
}

void ThrusterCommandHandler::publishStatus()
{
    abv_msgs::msg::AbvThrusterStatus status;
    status.thrusters = mThrusterCommander->getThrusterCommand();

    Eigen::Vector3d applied = mThrusterCommander->getAppliedThrustVector();
    status.fx = applied.x();
    status.fy = applied.y();
    status.tz = applied.z();

    RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvThrusterStatus>("abv/thrusters/status", status);
}

Eigen::Vector3d ThrusterCommandHandler::convertToBodyFrame(Eigen::Vector3d aVectorGlobal)
{
    auto state = mNavManager->getCurrentState();

    // yaw is element 2
    double yaw = state[2];

    // invert rotation relative to global
    Eigen::Matrix3d Rz;
    Rz << cos(yaw), sin(yaw), 0,
         -sin(yaw), cos(yaw), 0,
             0,        0,     1;

    return Rz * aVectorGlobal;
}
