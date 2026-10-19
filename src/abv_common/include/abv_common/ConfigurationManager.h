#ifndef CONFIGURATIONMANAGER_H
#define CONFIGURATIONMANAGER_H

#include "Configurations.h"
#include <yaml-cpp/yaml.h>

/**
 * @brief ConfigurationManager is a singleton responsible for loading the YAML config file and providing access
 *        to the data within it. 
 */
class ConfigurationManager
{
public:

    /**
     * @brief getInstance is the function used to obtain the singleton instance. 
     * 
     * @return a pointer to the singleton instance.
     */
    static ConfigurationManager* getInstance()
    {
        static ConfigurationManager instance;
        return &instance;
    }

    /**
     * @brief loadConfiguration loads the config file from the abv_bringup pkg 
     *        and parses its contents. Throws if any issues occur
     */
    void loadConfiguration();

    /**
     * @brief Getters for various sub-configs
     */
    GuidanceConfig& getGuidanceConfig() { return mGuidanceConfig; }
    NavigationConfig& getNavigationConfig() { return mNavigationConfig; }
    ControlConfig& getControlConfig() { return mControlConfig; }
    ControlLawConfig& getControlLawConfig() { return mControlLawConfig; }
    ThrusterConfig& getThrusterConfig() { return mThrusterConfig; }
    HeartbeatConfig& getHeartbeatConfig() { return mHeartbeatConfig; }
    TableViewConfig& getTableViewConfig() { return mTableViewConfig; }
    SceneConfig& getSceneConfig() { return mSceneConfig; }
    CollisionAvoidConfig& getCollisionAvoidConfig() { return mCollisionAvoidConfig; }

private: 
    /**
     * @brief private constructor. Does nothing.  
     */
    ConfigurationManager();
    /**
     * @brief private destructor. Does nothing.
     */
    ~ConfigurationManager();

    void parseGuidanceConfig(const YAML::Node& aNode);
    void parseNavigationConfig(const YAML::Node& aNode);
    void parseControlConfig(const YAML::Node& aNode);
    void parseControlLawConfig(const YAML::Node& aNode);
    void parseThrusterConfig(const YAML::Node& aNode);
    void parseHeartbeatConfig(const YAML::Node& aNode);
    void parseTableViewConfig(const YAML::Node& aNode);
    void parseSceneConfig(const YAML::Node& aNode);
    void parseCollisionAvoidConfig(const YAML::Node& aNode);

private:

    GuidanceConfig mGuidanceConfig;
    NavigationConfig mNavigationConfig;
    ControlConfig mControlConfig;
    ControlLawConfig mControlLawConfig;
    ThrusterConfig mThrusterConfig;
    HeartbeatConfig mHeartbeatConfig;
    TableViewConfig mTableViewConfig;
    SceneConfig mSceneConfig;
    CollisionAvoidConfig mCollisionAvoidConfig;
};

#endif

    