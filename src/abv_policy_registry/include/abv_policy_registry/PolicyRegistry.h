#ifndef POLICYREGISTRY_H
#define POLICYREGISTRY_H

#include <chrono>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "abv_msgs/msg/abv_heartbeat.hpp"
#include "abv_msgs/srv/abv_policy_switch.hpp"

/**
 * @brief PolicyRegistry discovers control-policy nodes (abv_control, abv_rl-cpp, ...)
 *        by subscribing to abv/policy/status (AbvHeartbeat, reused on a dedicated topic
 *        so every node_name seen there is inherently a policy - no name allowlist
 *        needed). Tracks per-policy liveness (stale after Heartbeat.StaleAfter with no
 *        message), re-broadcasts the active policy on abv/policy/active, publishes the
 *        known-policy list on abv/policy/registry, and hosts the abv/policy/switch
 *        service. Starts with no active policy - only an explicit switch request
 *        activates one. On a successful switch, publishes AbvControllerCommand{type:
 *        "clear"} on abv/controller/command to clear the current goal (no new goal is
 *        set).
 */
class PolicyRegistry
{
public:
    PolicyRegistry();
    ~PolicyRegistry();

private:
    void policyStatusCallback(abv_msgs::msg::AbvHeartbeat::SharedPtr aMsg);
    void switchCallback(const std::shared_ptr<abv_msgs::srv::AbvPolicySwitch::Request> aRequest,
                         std::shared_ptr<abv_msgs::srv::AbvPolicySwitch::Response> aResponse);
    void tick();

    bool isAlive(const std::string& aPolicyName) const;
    void publishActive();
    void publishRegistry();
    void publishClearGoal();

private:
    struct PolicyRecord
    {
        std::chrono::steady_clock::time_point mLastSeen;
        bool mAlive{true};
    };

    std::map<std::string, PolicyRecord> mPolicies;
    std::string mActivePolicy;

    rclcpp::TimerBase::SharedPtr mTimer;
    rclcpp::Service<abv_msgs::srv::AbvPolicySwitch>::SharedPtr mSwitchService;
};
#endif // POLICYREGISTRY_H
