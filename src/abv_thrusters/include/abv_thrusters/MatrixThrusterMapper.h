#ifndef MATRIXTHRUSTERMAPPER_H
#define MATRIXTHRUSTERMAPPER_H

#include "abv_thrusters/IThrusterMapper.hpp"
#include "abv_common/Configurations.h"

class MatrixThrusterMapper : public IThrusterMapper
{
public:
    MatrixThrusterMapper(const ThrusterConfig& aConfig, double aThrusterForce, double aMomentArm);

    ThrusterAllocation map(const Eigen::Vector3i& aThrustDir) override;

private:
    Eigen::Matrix<double, 3, 8> mB;
    Eigen::Matrix<double, 8, 3> mBpinv;
    double mAllocationThreshold;

    double mThrusterForce;
    double mMomentArm;
};
#endif //MATRIXTHRUSTERMAPPER_H
