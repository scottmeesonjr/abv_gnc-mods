#include "abv_common/ConfigurationManager.h"
#include "plog/Log.h"
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem> 
#include <array>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

ConfigurationManager::ConfigurationManager()
{
    // do nothing 
}

ConfigurationManager::~ConfigurationManager()
{
    // do nothing
}

void ConfigurationManager::loadConfiguration()
{   
    LOGD << R"(
        __    __   _______  __       __        ______               ___      .______   ____    ____ 
       |  |  |  | |   ____||  |     |  |      /  __  \             /   \     |   _  \  \   \  /   / 
       |  |__|  | |  |__   |  |     |  |     |  |  |  |           /  ^  \    |  |_)  |  \   \/   /  
       |   __   | |   __|  |  |     |  |     |  |  |  |          /  /_\  \   |   _  <    \      /   
       |  |  |  | |  |____ |  `----.|  `----.|  `--'  |  __     /  _____  \  |  |_)  |    \    /    
       |__|  |__| |_______||_______||_______| \______/  (_ )   /__/     \__\ |______/      \__/     
                                                         |/                                         
       )";

    std::string configFilePath = ament_index_cpp::get_package_share_directory("abv_bringup") + "/config/config.yaml";
    if (!std::filesystem::exists(configFilePath))
    {
        throw std::runtime_error("Missing configuration file at " + configFilePath); 
    }

    YAML::Node config = YAML::LoadFile(configFilePath); 
    std::stringstream s; 
    s << "Configuration: \n";
    s << YAML::Dump(config);
    LOGD << s.str();  

    // Parse configs 
    YAML::Node guidanceNode = config["Guidance"]; 
    parseGuidanceConfig(guidanceNode); 

    YAML::Node navigationNode = config["Navigation"]; 
    parseNavigationConfig(navigationNode); 

    YAML::Node controlNode = config["Control"];
    parseControlConfig(controlNode);

    YAML::Node controlLawNode = config["ControlLaw"];
    parseControlLawConfig(controlLawNode);

    YAML::Node thrustersNode = config["Thrusters"];
    parseThrusterConfig(thrustersNode);

    YAML::Node heartbeatNode = config["Heartbeat"];
    parseHeartbeatConfig(heartbeatNode);

    YAML::Node tableViewNode = config["TableView"];
    parseTableViewConfig(tableViewNode);

    YAML::Node sceneNode = config["Scene"];
    parseSceneConfig(sceneNode);

    YAML::Node collisionAvoidNode = config["CollisionAvoid"];
    parseCollisionAvoidConfig(collisionAvoidNode);

    // Docking block is optional: if it's missing, DockingConfig keeps its defaults
    YAML::Node dockingNode = config["Docking"];
    if (dockingNode)
    {
        parseDockingConfig(dockingNode);
    }
    else
    {
        LOGW << "No Docking block in config.yaml, using DockingConfig defaults";
    }
}

void ConfigurationManager::parseGuidanceConfig(const YAML::Node& aNode)
{
    mGuidanceConfig.mStateMachineRate = aNode["StateMachine"]["Rate"].as<int>();
    mGuidanceConfig.mWaypointTimeout = aNode["StateMachine"]["WaypointTimeout"].as<double>();
    mGuidanceConfig.mWaypointTimeoutToleranceScale = aNode["StateMachine"]["WaypointTimeoutToleranceScale"].as<double>();
}

void ConfigurationManager::parseNavigationConfig(const YAML::Node& aNode)
{
    mNavigationConfig.mInterface = aNode["Interface"].as<std::string>(); 
    mNavigationConfig.mRate = aNode["Rate"].as<int>(); 
    mNavigationConfig.mRigidBodyName = aNode["RigidBodyName"].as<std::string>(); 

    mNavigationConfig.mLocalIp = aNode["Network"]["Local"]["Ip"].as<std::string>();
    mNavigationConfig.mServerIp = aNode["Network"]["Server"]["Ip"].as<std::string>();

    mNavigationConfig.mMaxDeadReckonDuration = aNode["MaxDeadReckonDuration"].as<double>(0.5);
    mNavigationConfig.mSimulateDropout = aNode["SimulateDropout"].as<bool>(false);

    // SYS.01 velocity alert (m/s)
    mNavigationConfig.mVelocityAlertThreshold = aNode["VelocityAlertThreshold"].as<double>(0.125);
    mNavigationConfig.mVelocityAlertClear = aNode["VelocityAlertClear"].as<double>(0.115);

    if (mNavigationConfig.mVelocityAlertClear >= mNavigationConfig.mVelocityAlertThreshold)
    {
        throw std::runtime_error("Navigation.VelocityAlertClear must be less than VelocityAlertThreshold");
    }
}

void ConfigurationManager::parseControlConfig(const YAML::Node& aNode)
{
    mControlConfig.mStateMachineRate = aNode["StateMachine"]["Rate"].as<int>();
    mControlConfig.mPoseArrivalTol = ConfigUtils::parseVector3d(aNode["Arrival"]["Tolerance"]);
    mControlConfig.mArrivalDuration = aNode["Arrival"]["Duration"].as<double>();
    mControlConfig.mNavDataTimeout = aNode["NavDataTimeout"].as<double>(1.0);

    mControlConfig.mForce = aNode["Dynamics"]["ThrusterForce"].as<double>();
    mControlConfig.mMomentArm = aNode["Dynamics"]["MomentArm"].as<double>();
    mControlConfig.mMass = aNode["Dynamics"]["Mass"].as<double>();
    mControlConfig.mInertia = aNode["Dynamics"]["Inertia"].as<double>();

    std::string propagationMode = aNode["Dynamics"]["PropagationMode"].as<std::string>("internal");
    mControlConfig.mUseExternalPropagation = ("external" == propagationMode);
    mControlConfig.mExternalStateTopic = aNode["Dynamics"]["ExternalStateTopic"].as<std::string>("abv/sim/gazebo_state");
    mControlConfig.mWrenchCommandTopic = aNode["Dynamics"]["WrenchCommandTopic"].as<std::string>("gazebo/wrench_cmd");
    mControlConfig.mWrenchTargetLink = aNode["Dynamics"]["WrenchTargetLink"].as<std::string>("base_link");
}

void ConfigurationManager::parseControlLawConfig(const YAML::Node& aNode)
{
    mControlLawConfig.mLawType = aNode["Law"].as<std::string>("PID");

    mControlLawConfig.mKp = ConfigUtils::parseVector3d(aNode["Pid"]["Kp"]);
    mControlLawConfig.mKi = ConfigUtils::parseVector3d(aNode["Pid"]["Ki"]);
    mControlLawConfig.mKd = ConfigUtils::parseVector3d(aNode["Pid"]["Kd"]);

    // Velocity-goal gains. Optional: if the block is missing, the defaults in
    // ControlLawConfig are used (pose gains are NOT reused - see Configurations.h).
    YAML::Node velNode = aNode["VelocityPid"];
    if (velNode)
    {
        if (velNode["Kp"]) mControlLawConfig.mVelKp = ConfigUtils::parseVector3d(velNode["Kp"]);
        if (velNode["Ki"]) mControlLawConfig.mVelKi = ConfigUtils::parseVector3d(velNode["Ki"]);
        if (velNode["Kd"]) mControlLawConfig.mVelKd = ConfigUtils::parseVector3d(velNode["Kd"]);
    }
    else
    {
        LOGW << "No ControlLaw.VelocityPid block in config.yaml, using default velocity gains";
    }
}

void ConfigurationManager::parseThrusterConfig(const YAML::Node& aNode)
{
    mThrusterConfig.mSchmittTriggerOn = ConfigUtils::parseVector3d(aNode["InputDiscretization"]["On"]);
    mThrusterConfig.mSchmittTriggerOff = ConfigUtils::parseVector3d(aNode["InputDiscretization"]["Off"]);

    mThrusterConfig.mThrusterAllocationStrategy = aNode["AllocationStrategy"].as<std::string>("LookupTable");

    mThrusterConfig.mAllocationX   = ConfigUtils::parseIntVector(aNode["Allocation"]["x"]);
    mThrusterConfig.mAllocationY   = ConfigUtils::parseIntVector(aNode["Allocation"]["y"]);
    mThrusterConfig.mAllocationYaw = ConfigUtils::parseIntVector(aNode["Allocation"]["yaw"]);
    mThrusterConfig.mAllocationThreshold = aNode["Allocation"]["Threshold"].as<double>(0.1);

    mThrusterConfig.mThrusterDriverType = aNode["ThrusterDriver"]["Type"].as<std::string>();
    if("JETGPIO" == mThrusterConfig.mThrusterDriverType)
    {
        // parse GPIO pins
        mThrusterConfig.mGpioPins = ConfigUtils::parseIntVector(aNode["ThrusterDriver"]["GPIO"]["pins"]);
    }

    mThrusterConfig.mForce = aNode["Force"].as<double>();
    mThrusterConfig.mMomentArm = aNode["MomentArm"].as<double>();
}

void ConfigurationManager::parseHeartbeatConfig(const YAML::Node& aNode)
{
    mHeartbeatConfig.mRate = aNode["Rate"].as<double>();
    mHeartbeatConfig.mStaleAfter = aNode["StaleAfter"].as<double>();
    mHeartbeatConfig.mPingIntervalMs = aNode["PingIntervalMs"].as<int>();
}

void ConfigurationManager::parseTableViewConfig(const YAML::Node& aNode)
{
    mTableViewConfig.mWidth = aNode["Width"].as<double>();
    mTableViewConfig.mHeight = aNode["Height"].as<double>();
    mTableViewConfig.mRobotWidth = aNode["RobotWidth"].as<double>();
    mTableViewConfig.mRobotLength = aNode["RobotLength"].as<double>();
}

void ConfigurationManager::parseSceneConfig(const YAML::Node& aNode)
{
    mSceneConfig.mXMin = aNode["XMin"].as<double>();
    mSceneConfig.mXMax = aNode["XMax"].as<double>();
    mSceneConfig.mYMin = aNode["YMin"].as<double>();
    mSceneConfig.mYMax = aNode["YMax"].as<double>();
}

void ConfigurationManager::parseCollisionAvoidConfig(const YAML::Node& aNode)
{
    mCollisionAvoidConfig.mGridResolution = aNode["GridResolution"].as<double>();
    mCollisionAvoidConfig.mReplanRate = aNode["ReplanRate"].as<double>();
}

void ConfigurationManager::parseDockingConfig(const YAML::Node& aNode)
{
    auto readTriple = [&](const char* aKey, std::array<double, 3>& aOut)
    {
        if (!aNode[aKey]) return;  // keep default
        auto v = aNode[aKey].as<std::vector<double>>();
        if (v.size() != 3)
        {
            throw std::runtime_error(std::string("Docking.") + aKey + " must have 3 values");
        }
        aOut = {v[0], v[1], v[2]};
    };

    DockingConfig& c = mDockingConfig;
    readTriple("TargetPose", c.mTargetPose);
    readTriple("StartTolerance", c.mStartTolerance);
    c.mPortOffset = aNode["PortOffset"].as<double>(c.mPortOffset);
    c.mStandoffDistance = aNode["StandoffDistance"].as<double>(c.mStandoffDistance);
    c.mApproachVelocity = aNode["ApproachVelocity"].as<double>(c.mApproachVelocity);
    c.mApproachTimeout = aNode["ApproachTimeout"].as<double>(c.mApproachTimeout);
    c.mCaptureRange = aNode["CaptureRange"].as<double>(c.mCaptureRange);
    c.mDwellTime = aNode["DwellTime"].as<double>(c.mDwellTime);
    c.mCycles = aNode["Cycles"].as<int>(c.mCycles);
    c.mSerialPort = aNode["SerialPort"].as<std::string>(c.mSerialPort);
    c.mBaudRate = aNode["BaudRate"].as<int>(c.mBaudRate);
    c.mStatusTimeout = aNode["StatusTimeout"].as<double>(c.mStatusTimeout);
    c.mSimulateContact = aNode["SimulateContact"].as<bool>(c.mSimulateContact);
    c.mTargetHalfWidth = aNode["TargetHalfWidth"].as<double>(c.mTargetHalfWidth);

    // Warn only, so you can still deliberately run outside the band (e.g. to trip the alert).
    if (c.mApproachVelocity < 0.05 || c.mApproachVelocity > 0.10)
    {
        LOGW << "Docking.ApproachVelocity " << c.mApproachVelocity
             << " m/s is outside the SYS.10 band (0.05-0.10 m/s)";
    }
    if (c.mStandoffDistance <= c.mCaptureRange)
    {
        throw std::runtime_error("Docking.StandoffDistance must be greater than CaptureRange");
    }
}
