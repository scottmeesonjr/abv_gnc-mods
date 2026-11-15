
#include "abv_common/RosTopicManager.h"
#include "abv_commander/CommanderComms.h"
#include "abv_msgs/msg/dock_command.hpp"
#include "std_msgs/msg/string.hpp"

CommanderComms::CommanderComms()
{
    rclcpp::init(0, nullptr); 
}

CommanderComms::~CommanderComms()
{

}

bool CommanderComms::start()
{
    auto topicManager = RosTopicManager::getInstance(); 

    topicManager->createPublisher<abv_msgs::msg::AbvControllerCommand>("abv/controller/command");
    topicManager->createPublisher<abv_msgs::msg::AbvGuidanceCommand>("abv/guidance/command"); 
    topicManager->createPublisher<abv_msgs::msg::DockCommand>("abv/dock/cmd");
    topicManager->createPublisher<std_msgs::msg::String>("abv/test/condition");
    topicManager->createPublisher<std_msgs::msg::String>("abv/test/run_name");
    
    topicManager->spinNode(); 

    return true; 
}

bool CommanderComms::stop()
{
    rclcpp::shutdown(); 
    return true;
}