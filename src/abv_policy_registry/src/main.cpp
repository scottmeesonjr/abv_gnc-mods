
#include <cstdio>
#include "abv_common/RosTopicManager.h"
#include "abv_common/DataLogger.h"
#include "abv_common/SignalHandler.hpp"
#include "abv_common/ConfigurationManager.h"
#include "abv_common/HeartbeatPublisher.h"
#include "abv_policy_registry/PolicyRegistry.h"

int main()
{
    std::signal(SIGINT, signalHandler);

    DataLogger::get().createMainLog("abv_policy_registry");
    ConfigurationManager::getInstance()->loadConfiguration();

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("abv_policy_registry");
    RosTopicManager::getInstance()->spinNode();
    HeartbeatPublisher heartbeat("policy_registry");

    PolicyRegistry policyRegistry;

    while(rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    rclcpp::shutdown();
}
