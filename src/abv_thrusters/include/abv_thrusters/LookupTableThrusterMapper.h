#ifndef LOOKUPTABLETHRUSTERMAPPER_H
#define LOOKUPTABLETHRUSTERMAPPER_H

#include "abv_thrusters/IThrusterMapper.hpp"

class LookupTableThrusterMapper : public IThrusterMapper
{
public:
    LookupTableThrusterMapper(double aThrusterForce, double aMomentArm);

    ThrusterAllocation map(const Eigen::Vector3i& aThrustDir) override;

private:
    Eigen::Matrix<int, 3, 27> mMatrixOfThrustDirCombinations;

    double mThrusterForce;
    double mMomentArm;
};
#endif //LOOKUPTABLETHRUSTERMAPPER_H
