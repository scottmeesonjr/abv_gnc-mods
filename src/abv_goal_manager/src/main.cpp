
#include <cstdio>
#include "abv_goal_manager/CommandHandler.h"
#include "abv_goal_manager/StateMachine.h"
#include "abv_goal_manager/GoalManager.h"
#include "abv_common/DataLogger.h"
#include "abv_common/SignalHandler.hpp"
#include "abv_common/ConfigurationManager.h"
#include "abv_common/HeartbeatPublisher.h"

int main()
{
    std::signal(SIGINT, signalHandler);

    DataLogger::get().createMainLog("abv_goal_manager");
    ConfigurationManager::getInstance()->loadConfiguration();

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("abv_goal_manager");
    RosTopicManager::getInstance()->spinNode();
    HeartbeatPublisher heartbeat("goal_manager");

    std::shared_ptr<GoalManager> abv = std::make_shared<GoalManager>();

    std::shared_ptr<StateMachine> msm = std::make_shared<StateMachine>(abv);
    std::unique_ptr<CommandHandler> cmdHandler = std::make_unique<CommandHandler>(msm, abv);

    msm->run();

    rclcpp::shutdown();
}
