
#include "abv_guidance/RosCommandSource.h"
#include "abv_common/RosTopicManager.h"

RosCommandSource::RosCommandSource(ICommandSink& aSink) : ICommandSource(aSink)
{

}

RosCommandSource::~RosCommandSource()
{

}

void RosCommandSource::listen()
{
    RosTopicManager::getInstance()->createSubscriber<abv_msgs::msg::AbvGuidanceCommand>("abv/guidance/command", 
                                                                                         std::bind(&RosCommandSource::commandCallback, 
                                                                                                   this, 
                                                                                                   std::placeholders::_1));

    RosTopicManager::getInstance()->spinNode();                                                                                        
    while(!RosTopicManager::getInstance()->isROSInitialized())
    {
        // wait to ensure ros is initialized 
    }
}

void RosCommandSource::commandCallback(abv_msgs::msg::AbvGuidanceCommand::SharedPtr aCmd)
{
    abv_msgs::msg::AbvVec3 goal = aCmd->goal_state.position; 
    Waypoint wp(goal.x, goal.y, goal.yaw, "pose");  // TODO: support sending vel commands
    // carried through for path types that use it ("dock": velocity.x = cycles, 0 = config)
    abv_msgs::msg::AbvVec3 gv = aCmd->goal_state.velocity;
    wp.mVel << gv.x, gv.y, gv.yaw;

    Command cmd; 
    cmd.mType = aCmd->type; 
    cmd.mDuration = aCmd->duration;
    cmd.mGoal = wp; 

    // TODO: when adding support for vel commands, update this too
    auto tol = aCmd->arrival_tolerance.position; 
    cmd.mArrivalTol << tol.x, tol.y, tol.yaw; 

    mCommandSink.onCommand(cmd); 
}

void RosCommandSource::stop()
{

}
