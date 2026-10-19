
#include "abv_control/PidControlLaw.h"
#include "abv_common/ConfigurationManager.h"

PidControlLaw::PidControlLaw() : mConfig(ConfigurationManager::getInstance()->getControlLawConfig())
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
    actionCtx.controlInput = PID(ctx.error);
    actionCtx.isGlobal = true;
    return true;
}

void PidControlLaw::reset()
{
    mPrevTime = std::chrono::steady_clock::now();
    mPrevPoseError = Eigen::Vector3d::Zero();
    mPoseErrorIntegral = Eigen::Vector3d::Zero();
}

Eigen::Vector3d PidControlLaw::PID(Eigen::Vector3d aPoseError)
{
    using namespace std::chrono;

    Eigen::Vector3d poseErrorDeriv;
    Eigen::Vector3d poseErrorIntegral;
    Eigen::Vector3d controlInput = Eigen::Vector3d::Zero();

    double dt = duration_cast<milliseconds>(steady_clock::now() - mPrevTime).count() / 1000.0;

    for (int i = 0; i < 3; i++)
    {
        double error = aPoseError[i];
        double deriv = (error - mPrevPoseError[i]) / dt;
        mPoseErrorIntegral[i] += error * dt;

        controlInput[i] = mConfig.mKp[i] * error +
                          mConfig.mKi[i] * mPoseErrorIntegral[i] +
                          mConfig.mKd[i] * deriv;
    }

    mPrevPoseError = aPoseError;
    mPrevTime = steady_clock::now();

    return controlInput;
}
