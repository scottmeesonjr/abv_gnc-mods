#ifndef PIDCONTROLLAW_H
#define PIDCONTROLLAW_H

#include <eigen3/Eigen/Dense>
#include "abv_common/Configurations.h"
#include "abv_control/IControlLaw.hpp"
#include <chrono>


class PidControlLaw : public IControlLaw
{
public:
    PidControlLaw();
    ~PidControlLaw();

    bool computeAction(const ControlContext& ctx, ActionContext& actionCtx) override;
    void reset() override;

private:
    ControlLawConfig mConfig;
    std::chrono::steady_clock::time_point mPrevTime;
    Eigen::Vector3d mPrevPoseError;
    Eigen::Vector3d mPoseErrorIntegral;

    // Pose and velocity goals use separate gains (ControlLaw.Pid vs
    // ControlLaw.VelocityPid). The error being fed in has different units in
    // each mode, so the derivative/integral state is reset whenever the goal
    // type changes to keep one mode's history from kicking the other.
    GoalType mLastGoalType;

    Eigen::Vector3d PID(Eigen::Vector3d aPoseError, const Eigen::Vector3d& aKp,
                        const Eigen::Vector3d& aKi, const Eigen::Vector3d& aKd);

};
#endif //PIDCONTROLLAW_H
