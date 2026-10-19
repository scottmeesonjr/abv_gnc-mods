
#include "abv_common/HeartbeatPublisher.h"
#include "abv_common/RosTopicManager.h"
#include "abv_common/ConfigurationManager.h"
#include "abv_msgs/msg/abv_heartbeat.hpp"

HeartbeatPublisher::HeartbeatPublisher(const std::string& aNodeName, const std::string& aTopic) : mNodeName(aNodeName), mTopic(aTopic)
{
    RosTopicManager::getInstance()->createPublisher<abv_msgs::msg::AbvHeartbeat>(mTopic);

    double rateHz = ConfigurationManager::getInstance()->getHeartbeatConfig().mRate;
    auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / rateHz));

    mTimer = RosTopicManager::getInstance()->create_wall_timer(period, std::bind(&HeartbeatPublisher::tick, this));
}

HeartbeatPublisher::~HeartbeatPublisher()
{
}

void HeartbeatPublisher::tick()
{
    abv_msgs::msg::AbvHeartbeat msg;
    msg.set__node_name(mNodeName);
    RosTopicManager::getInstance()->publishMessage(mTopic, msg);
}
