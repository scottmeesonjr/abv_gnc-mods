
#include "abv_thrusters/MatrixThrusterMapper.h"
#include <sstream>
#include <stdexcept>

MatrixThrusterMapper::MatrixThrusterMapper(const ThrusterConfig& aConfig, double aThrusterForce, double aMomentArm) :
    mThrusterForce(aThrusterForce), mMomentArm(aMomentArm)
{
    if(aConfig.mAllocationX.size() != 8 || aConfig.mAllocationY.size() != 8 || aConfig.mAllocationYaw.size() != 8)
    {
        throw std::runtime_error("Thrusters.Allocation rows must each have exactly 8 entries");
    }

    for(int i = 0; i < 8; i++)
    {
        mB(0, i) = aConfig.mAllocationX[i];
        mB(1, i) = aConfig.mAllocationY[i];
        mB(2, i) = aConfig.mAllocationYaw[i];
    }

    mBpinv = mB.completeOrthogonalDecomposition().pseudoInverse();
    mAllocationThreshold = aConfig.mAllocationThreshold;
}

ThrusterAllocation MatrixThrusterMapper::map(const Eigen::Vector3i& aThrustDir)
{
    Eigen::Vector3d wrench = aThrustDir.cast<double>();
    Eigen::VectorXd u = mBpinv * wrench;

    Eigen::Vector3d input = Eigen::Vector3d::Zero();
    std::stringstream ss;
    for(int i = 0; i < 8; i++)
    {
        bool fires = u[i] > mAllocationThreshold;
        ss << (fires ? "1" : "0");
        if(fires)
        {
            input += mB.col(i);
        }
    }

    ThrusterAllocation result;
    result.mThrusterCommand = ss.str();
    result.mAppliedThrustVector[0] = input[0] * mThrusterForce;
    result.mAppliedThrustVector[1] = input[1] * mThrusterForce;
    result.mAppliedThrustVector[2] = input[2] * (mMomentArm * mThrusterForce);
    return result;
}
