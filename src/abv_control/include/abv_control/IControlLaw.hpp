#ifndef ICONTROLLAW_HPP
#define ICONTROLLAW_HPP

#include <Eigen/Dense>
#include "abv_control/GoalType.hpp"

struct ControlContext
{
    // Inputs
    Eigen::Vector3d  currentPose;
    Eigen::Vector3d  currentVelocity;
    Eigen::Vector3d  goal;
    Eigen::Vector3d  error;
    GoalType         goalType;
};
struct ActionContext
{
    // Outputs
    Eigen::Vector3d  controlInput;
    bool             isGlobal;
};

// Interface for any classical/model-based control law hosted by this node
// (PID today; LQR/MPC etc. can be added later as additional implementations
// selected via ControlLaw.Law in config, without a new node).
class IControlLaw
{
public:
    virtual ~IControlLaw() = default;
    virtual bool computeAction(const ControlContext& ctx, ActionContext& actionCtx) = 0;

    // Called by ControlLawHandler when this law transitions from inactive to
    // active (a policy switch activated it) - clears any time-dependent
    // internal state (integral accumulators, previous-error/previous-time
    // for derivative/dt) so a long gap while inactive doesn't produce a
    // huge dt on the first tick back. Default no-op for stateless laws.
    virtual void reset() {}
};
#endif //ICONTROLLAW_HPP
