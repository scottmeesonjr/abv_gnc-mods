#ifndef HEARTBEATPUBLISHER_H
#define HEARTBEATPUBLISHER_H

#include <string>
#include <rclcpp/rclcpp.hpp>

// Publishes an AbvHeartbeat{node_name} on aTopic (defaults to the shared
// "abv/heartbeat" topic) at the rate configured in Heartbeat.Rate, for as
// long as this object is kept alive. Construct one of these in a node's
// main() before the blocking run()/spin call so it lives for the process's
// lifetime. A node that wants to announce itself on a different topic (e.g.
// a control-policy node heartbeating on "abv/policy/status" so
// abv_policy_registry can discover it) can construct a second instance with
// an explicit topic, alongside its regular "abv/heartbeat" one.
class HeartbeatPublisher
{
public:
    explicit HeartbeatPublisher(const std::string& aNodeName, const std::string& aTopic = "abv/heartbeat");
    ~HeartbeatPublisher();

private:
    void tick();

    std::string mNodeName;
    std::string mTopic;
    rclcpp::TimerBase::SharedPtr mTimer;
};
#endif //HEARTBEATPUBLISHER_H
