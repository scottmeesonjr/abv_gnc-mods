
#include "abv_control/PidControlLaw.h"
#include "abv_common/ConfigurationManager.h"

PidControlLaw::PidControlLaw() : mConfig(ConfigurationManager::getInstance()->getControlLawConfig()),
    mLastGoalType(GoalType::NUM_TYPES)
{
    mPrevTime = std::chrono::steady_clock::now();
    mPrevPoseError = Eigen::Vector3d::Zero();
    mPoseErrorIntegral = Eigen::Vector3d::Zero();
}

PidControlLaw::~PidControlLaw()
{

}

bool PidControlLaw::computeAction(const ControlContext& ctx, ActionContext& actionCtx)
{
    if(ctx.goalType != mLastGoalType)
    {
        // switching between pose and velocity control: the previous error was
        // in different units, so start the derivative/integral fresh
        reset();
        mPrevPoseError = ctx.error;
        mLastGoalType = ctx.goalType;
    }

    if(GoalType::VELOCITY == ctx.goalType)
    {
        actionCtx.controlInput = PID(ctx.error, mConfig.mVelKp, mConfig.mVelKi, mConfig.mVelKd);
    }
    else
    {
        actionCtx.controlInput = PID(ctx.error, mConfig.mKp, mConfig.mKi, mConfig.mKd);
    }
    actionCtx.isGlobal = true;
    return true;
}

void PidControlLaw::reset()
{
    mPrevTime = std::chrono::steady_clock::now();
    mPrevPoseError = Eigen::Vector3d::Zero();
    mPoseErrorIntegral = Eigen::Vector3d::Zero();
}

Eigen::Vector3d PidControlLaw::PID(Eigen::Vector3d aPoseError, const Eigen::Vector3d& aKp,
                                   const Eigen::Vector3d& aKi, const Eigen::Vector3d& aKd)
{
    using namespace std::chrono;

    Eigen::Vector3d controlInput = Eigen::Vector3d::Zero();

    double dt = duration_cast<milliseconds>(steady_clock::now() - mPrevTime).count() / 1000.0;
    if(dt <= 0.0)
    {
        dt = 1e-3; // two calls in the same millisecond: avoid dividing by zero
    }

    for (int i = 0; i < 3; i++)
    {
        double error = aPoseError[i];
        double deriv = (error - mPrevPoseError[i]) / dt;
        mPoseErrorIntegral[i] += error * dt;

        controlInput[i] = aKp[i] * error +
                          aKi[i] * mPoseErrorIntegral[i] +
                          aKd[i] * deriv;
    }

    mPrevPoseError = aPoseError;
    mPrevTime = steady_clock::now();

    return controlInput;
}
