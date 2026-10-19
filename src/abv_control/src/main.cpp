
#include <cstdio>
#include "abv_common/RosTopicManager.h"
#include "abv_common/DataLogger.h"
#include "abv_common/SignalHandler.hpp"
#include "abv_common/ConfigurationManager.h"
#include "abv_common/HeartbeatPublisher.h"
#include "abv_control/ControlLawHandler.h"

int main()
{
    std::signal(SIGINT, signalHandler);

    DataLogger::get().createMainLog("abv_control");
    ConfigurationManager::getInstance()->loadConfiguration();

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("abv_control");
    RosTopicManager::getInstance()->spinNode();
    HeartbeatPublisher heartbeat("control");
    HeartbeatPublisher policyHeartbeat("control", "abv/policy/status");

    ControlLawHandler controlLawHandler;

    while(rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    rclcpp::shutdown();
}
