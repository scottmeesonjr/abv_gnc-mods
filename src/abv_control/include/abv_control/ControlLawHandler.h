#ifndef CONTROLLAWHANDLER_H
#define CONTROLLAWHANDLER_H

#include <memory>
#include <string>
#include "abv_control/IControlLaw.hpp"
#include "abv_msgs/msg/abv_control_context.hpp"
#include "abv_msgs/msg/abv_policy_active.hpp"

/**
 * @brief ControlLawHandler subscribes to abv/controller/context, runs the
 *        configured IControlLaw against each message, and publishes the
 *        resulting control effort on abv/thrusters/command. Instantiates
 *        the IControlLaw implementation selected by ControlLawConfig.mLawType
 *        (PID today; a new law is a new IControlLaw implementation plus one
 *        more branch in the constructor).
 *
 *        Also subscribes to abv/policy/active and gates contextCallback on
 *        it: when this node isn't the active policy, it skips computing AND
 *        publishing entirely (not just relying on abv_thrusters' downstream
 *        arbiter), so a switched-away policy genuinely stops running rather
 *        than silently computing in the background. mControlLaw->reset() is
 *        called on the inactive->active transition so stale time-dependent
 *        state (PID integral/dt) from before the gap doesn't resurface.
 */
class ControlLawHandler
{
public:
    ControlLawHandler();
    ~ControlLawHandler();

private:
    void contextCallback(abv_msgs::msg::AbvControlContext::SharedPtr aCtxMsg);
    void policyActiveCallback(abv_msgs::msg::AbvPolicyActive::SharedPtr aMsg);

private:
    std::unique_ptr<IControlLaw> mControlLaw;
    const std::string mPolicyName{"control"};
    bool mIsActive{false};
};
#endif // CONTROLLAWHANDLER_H
