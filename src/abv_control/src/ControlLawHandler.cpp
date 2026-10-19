#include "abv_control/ControlLawHandler.h"
#include "abv_control/PidControlLaw.h"

#include "abv_common/RosTopicManager.h"
#include "abv_common/ConfigurationManager.h"
#include "abv_msgs/msg/abv_thruster_command.hpp"
#include "plog/Log.h"

namespace
{
    Eigen::Vector3d toEigen(const abv_msgs::msg::AbvVec3& aVec)
    {
        return Eigen::Vector3d(aVec.x, aVec.y, aVec.yaw);
    }

    abv_msgs::msg::AbvVec3 toMsg(const Eigen::Vector3d& aVec)
    {
        abv_msgs::msg::AbvVec3 msg;
        msg.x = aVec.x();
        msg.y = aVec.y();
        msg.yaw = aVec.z();
        return msg;
    }
}

ControlLawHandler::ControlLawHandler()
{
    auto& config = ConfigurationManager::getInstance()->getControlLawConfig();

    if("LQR" == config.mLawType || "lqr" == config.mLawType ||
       "MPC" == config.mLawType || "mpc" == config.mLawType)
    {
        LOGW << "ControlLaw.Law '" << config.mLawType << "' is not implemented yet, defaulting to PID";
        mControlLaw = std::make_unique<PidControlLaw>();
    }
    else
    {
        if("PID" != config.mLawType && "pid" != config.mLawType)
        {
            LOGW << "Unknown ControlLaw.Law '" << config.mLawType << "', defaulting to PID";
        }
        mControlLaw = std::make_unique<PidControlLaw>();
    }

    auto topicManager = RosTopicManager::getInstance();
    topicManager->createPublisher<abv_msgs::msg::AbvThrusterCommand>("abv/thrusters/command");
    topicManager->createSubscriber<abv_msgs::msg::AbvControlContext>("abv/controller/context",
                                    std::bind(&ControlLawHandler::contextCallback, this, std::placeholders::_1));
    topicManager->createSubscriber<abv_msgs::msg::AbvPolicyActive>("abv/policy/active",
                                    std::bind(&ControlLawHandler::policyActiveCallback, this, std::placeholders::_1));

    topicManager->spinNode();
}

ControlLawHandler::~ControlLawHandler()
{
}

void ControlLawHandler::policyActiveCallback(abv_msgs::msg::AbvPolicyActive::SharedPtr aMsg)
{
    bool wasActive = mIsActive;
    mIsActive = (aMsg->policy_name == mPolicyName);

    if(mIsActive && !wasActive)
    {
        mControlLaw->reset();
    }
}

void ControlLawHandler::contextCallback(abv_msgs::msg::AbvControlContext::SharedPtr aCtxMsg)
{
    if(!mIsActive)
    {
        return;
    }

    ControlContext ctx;
    ctx.currentPose = toEigen(aCtxMsg->pose);
    ctx.currentVelocity = toEigen(aCtxMsg->velocity);
    ctx.goal = toEigen(aCtxMsg->goal);
    ctx.error = toEigen(aCtxMsg->error);
    ctx.goalType = (abv_msgs::msg::AbvControlContext::VELOCITY == aCtxMsg->goal_type) ? GoalType::VELOCITY : GoalType::POSE;

    ActionContext actionCtx;
    if(!mControlLaw->computeAction(ctx, actionCtx))
    {
        LOGE << "Failed to compute control action";
        return;
    }

    abv_msgs::msg::AbvThrusterCommand cmd;
    cmd.type = "direction";
    cmd.is_global = actionCtx.isGlobal;
    cmd.data = toMsg(actionCtx.controlInput);
    cmd.policy_name = mPolicyName;

    RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvThrusterCommand>("abv/thrusters/command", cmd);
}
