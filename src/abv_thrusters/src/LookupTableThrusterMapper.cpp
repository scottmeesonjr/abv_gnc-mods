
#include "abv_thrusters/LookupTableThrusterMapper.h"

LookupTableThrusterMapper::LookupTableThrusterMapper(double aThrusterForce, double aMomentArm) :
    mThrusterForce(aThrusterForce), mMomentArm(aMomentArm)
{
    mMatrixOfThrustDirCombinations << 1, -1, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, -1, 0, 0, 0, 0, 1, 1, 1, -1, 1, -1, -1, -1, 0,
			                          0, 0, 1, -1, 0, 0, 1, -1, 1, -1, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 0,
			                          0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, 0;
}

ThrusterAllocation LookupTableThrusterMapper::map(const Eigen::Vector3i& aThrustDir)
{
    ThrusterAllocation result;

    // calculate the thruster command sequence based on the control input
    if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 0))) // +x
    {
        result.mThrusterCommand = "00000011";
        result.mAppliedThrustVector << 2 * mThrusterForce, 0, 0;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 1))) // -x
    {
        result.mThrusterCommand = "00110000";
        result.mAppliedThrustVector << -2 * mThrusterForce, 0, 0;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 2))) // +y
    {
        result.mThrusterCommand = "11000000";
        result.mAppliedThrustVector << 0, 2 * mThrusterForce, 0;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 3))) // -y
    {
        result.mThrusterCommand = "00001100";
        result.mAppliedThrustVector << 0, -2 * mThrusterForce, 0;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 4))) // +phi
    {
        result.mThrusterCommand = "01000100";
        result.mAppliedThrustVector << 0, 0, 2*mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 5))) // -phi
    {
        result.mThrusterCommand = "10001000";
        result.mAppliedThrustVector << 0, 0, -2*mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 6))) // +x +y
    {
        result.mThrusterCommand = "01000010";
        result.mAppliedThrustVector << mThrusterForce, mThrusterForce, 0;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 7))) // +x -y
    {
        result.mThrusterCommand = "00001001";
        result.mAppliedThrustVector << mThrusterForce, -mThrusterForce, 0;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 8))) // -x +y
    {
        result.mThrusterCommand = "10010000";
        result.mAppliedThrustVector << -mThrusterForce, mThrusterForce, 0;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 9))) // -x -y
    {
        result.mThrusterCommand = "00100100";
        result.mAppliedThrustVector << -mThrusterForce, -mThrusterForce, 0;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 10))) // +x +phi
    {
        result.mThrusterCommand = "00000001";
        result.mAppliedThrustVector << mThrusterForce, 0, mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 11))) // +x -phi
    {
        result.mThrusterCommand = "00000010";
        result.mAppliedThrustVector << mThrusterForce, 0, -mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 12))) // -x +phi
    {
        result.mThrusterCommand = "00010000";
        result.mAppliedThrustVector << -mThrusterForce, 0, mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 13))) // -x -phi
    {
        result.mThrusterCommand = "00100000";
        result.mAppliedThrustVector << -mThrusterForce, 0, -mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 14))) // +y +phi
    {
        result.mThrusterCommand = "01000000";
        result.mAppliedThrustVector << 0, mThrusterForce, mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 15))) // +y -phi
    {
        result.mThrusterCommand = "10000000";
        result.mAppliedThrustVector << 0, mThrusterForce, -mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 16))) // -y +phi
    {
        result.mThrusterCommand = "00000100";
        result.mAppliedThrustVector << 0, -mThrusterForce, mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 17))) // -y -phi
    {
        result.mThrusterCommand = "00001000";
        result.mAppliedThrustVector << 0, -mThrusterForce, -mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 18))) // +x +y +phi
    {
        result.mThrusterCommand = "01000001";
        result.mAppliedThrustVector << mThrusterForce, mThrusterForce, mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 19))) // +x +y -phi
    {
        result.mThrusterCommand = "10000010";
        result.mAppliedThrustVector << mThrusterForce, mThrusterForce, -mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 20))) // +x -y +phi
    {
        result.mThrusterCommand = "00000101";
        result.mAppliedThrustVector << mThrusterForce, -mThrusterForce, mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 21))) // -x +y +phi
    {
        result.mThrusterCommand = "01010000";
        result.mAppliedThrustVector << -mThrusterForce, mThrusterForce, mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 22))) // +x -y -phi
    {
        result.mThrusterCommand = "00001010";
        result.mAppliedThrustVector << mThrusterForce, mThrusterForce, -mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 23))) // -x +y -phi
    {
        result.mThrusterCommand = "10100000";
        result.mAppliedThrustVector << -mThrusterForce, mThrusterForce, -mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 24))) // -x -y +phi
    {
        result.mThrusterCommand = "00010100";
        result.mAppliedThrustVector << -mThrusterForce, -mThrusterForce, mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 25))) // -x -y -phi
    {
        result.mThrusterCommand = "00101000";
        result.mAppliedThrustVector << -mThrusterForce, -mThrusterForce, -mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 26))) // nothing
    {
        result.mThrusterCommand = "00000000";
        result.mAppliedThrustVector << 0, 0, 0;
    }

    return result;
}
