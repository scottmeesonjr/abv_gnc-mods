
#include <cstdio>
#include "abv_common/RosTopicManager.h"
#include "abv_common/ConfigurationManager.h"
#include "abv_common/HeartbeatPublisher.h"
#include "abv_bridge/NavigationConvertor.h"
#include "abv_bridge/ControllerStatusConvertor.h"
#include "abv_bridge/WaypointConvertor.h"
#include "abv_bridge/GazeboStateConvertor.h"

int main()
{
    ConfigurationManager::getInstance()->loadConfiguration();

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("abv_bridge");
    RosTopicManager::getInstance()->spinNode();

    // ABV -> Autonomy
    NavigationConvertor nav("abv/state", "robot/state");
    ControllerStatusConvertor status("abv/controller/status", "robot/vehicle/controller_status");

    // Autonomy -> ABV
    WaypointConvertor waypoint("robot/vehicle/waypoint", "abv/guidance/command");

    // Gazebo -> ABV Sim (external propagation feedback)
    GazeboStateConvertor gazeboState("mujoco/robot/state", "abv/sim/gazebo_state");

    HeartbeatPublisher heartbeat("bridge");

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
    }

    rclcpp::shutdown(); 
    return 0; 
}