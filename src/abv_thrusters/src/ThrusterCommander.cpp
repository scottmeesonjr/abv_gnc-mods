#include "abv_thrusters/ThrusterCommander.h"
#include "abv_thrusters/LookupTableThrusterMapper.h"
#include "abv_thrusters/MatrixThrusterMapper.h"
#include "plog/Log.h"

#if defined(ARCH_X86)
    // not really arch specific but im too lazy to make configurable rn
    #include "abv_thrusters/UdpThrusterDriver.h"
    using ThrusterDriverImpl = UdpThrusterDriver;
#elif defined(ARCH_ARM)
    #include "abv_thrusters/GpioThrusterDriver.h"
    using ThrusterDriverImpl = GpioThrusterDriver;
#else
    #error "Unsupported architecture. Define ARCH_X86 or ARCH_ARM."
#endif


ThrusterCommander::ThrusterCommander() :
    mThrusterCommand("00000000"),
    mConfig(ConfigurationManager::getInstance()->getThrusterConfig()),
    mThrusterDriver(std::make_unique<ThrusterDriverImpl>(mConfig.mGpioPins))
{
    mThrusterDriver->init();

    mThrusterForce = mConfig.mForce;
    mMomentArm = mConfig.mMomentArm;
    mAppliedThrustVector = Eigen::Vector3d::Zero();

    if("Matrix" == mConfig.mThrusterAllocationStrategy || "matrix" == mConfig.mThrusterAllocationStrategy)
    {
        mThrusterMapper = std::make_unique<MatrixThrusterMapper>(mConfig, mThrusterForce, mMomentArm);
    }
    else
    {
        if("LookupTable" != mConfig.mThrusterAllocationStrategy && "lookuptable" != mConfig.mThrusterAllocationStrategy)
        {
            LOGW << "Unknown ThrusterAllocationStrategy '" << mConfig.mThrusterAllocationStrategy << "', defaulting to LookupTable";
        }
        mThrusterMapper = std::make_unique<LookupTableThrusterMapper>(mThrusterForce, mMomentArm);
    }
}

ThrusterCommander::~ThrusterCommander()
{

}

void ThrusterCommander::commandThrusters(const std::string& aThrustersCmd)
{
    {
        std::lock_guard<std::mutex> lock(mThrusterCommandMutex);
        mThrusterCommand = aThrustersCmd;
    }

    mThrusterDriver->send(aThrustersCmd);
}

void ThrusterCommander::command(Eigen::Vector3d aControlInput)
{
    // convert control input to thruster dir vector
    Eigen::Vector3i thrustDirVector = convertToThrustVector(aControlInput);

    // convert thrust dir vector into thruster combination
    ThrusterAllocation allocation = mThrusterMapper->map(thrustDirVector);

    {
        std::lock_guard<std::mutex> lock(mThrusterCommandMutex);
        mThrusterCommand = allocation.mThrusterCommand;
        mAppliedThrustVector = allocation.mAppliedThrustVector;
    }

    mThrusterDriver->send(mThrusterCommand);
}

Eigen::Vector3i ThrusterCommander::convertToThrustVector(Eigen::Vector3d aControlInput)
{
    Eigen::Vector3i thrustDir;
    Eigen::Vector3d uOn = mConfig.mSchmittTriggerOn;
    Eigen::Vector3d uOff = mConfig.mSchmittTriggerOff;

    for(int i = 0; i < 3; i++)
    {
        // determine the thrust direction based on the control input
        if (aControlInput[i] >= uOn[i])
		{
			thrustDir[i] = 1;
		}
		else if (aControlInput[i] < -uOn[i])
		{
			thrustDir[i] = -1;
		}
		else if (aControlInput[i] <= uOff[i] && aControlInput[i] >= -uOff[i])
		{
			thrustDir[i] = 0;
		}
		else
		{
			thrustDir[i] = 0;
		}

    }

    return thrustDir;
}

Eigen::Vector3d ThrusterCommander::getAppliedThrustVector()
{
    std::lock_guard<std::mutex> lock(mThrusterCommandMutex);
    return mAppliedThrustVector;
}

std::string ThrusterCommander::getThrusterCommand()
{
    std::lock_guard<std::mutex> lock(mThrusterCommandMutex);
    return mThrusterCommand;
}
