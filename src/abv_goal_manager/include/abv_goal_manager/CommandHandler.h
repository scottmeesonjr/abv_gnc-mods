#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H

#include "abv_common/RosTopicManager.h"

#include "abv_msgs/msg/abv_controller_command.hpp"
#include "abv_goal_manager/StateMachine.h"
#include "abv_goal_manager/GoalManager.h"

/**
 * @brief CommandHandler is responsible for receiving external commands via ROS2 topics and changing
 *        the active state of the state machine to perform the desired action. 
 */
class CommandHandler
{

public:
    
    /**
     * @brief CommandHandler constructor, initializes the ROS2 topics and spins the ROS2 node
     * 
     * @param msm a pointer to the state machine. Used to change the active state for performing an action
     * @param abv a pointer to the vehicle, use to pass data to the vehicle class 
     */
    CommandHandler(std::shared_ptr<StateMachine> msm, std::shared_ptr<GoalManager> abv);

    /**
     * @brief CommandHandler destructor, does nothing. 
     */
    ~CommandHandler();

    /**
     * @brief CommandType enum, used to determine what type of command has been received
     */
    enum class CommandType
    {
        THRUSTER,
        POSE,
        VELOCITY,
        STOP,
        DIRECTION,
        CLEAR,
        NUM_TYPES
    };

    /**
     * @brief commandCallback the callback invoked when a command is received
     * 
     * @param aCmdMsg a pointer to the data contained in the message
     */
    void commandCallback(abv_msgs::msg::AbvControllerCommand::SharedPtr aCmdMsg); 

    /**
     * @brief toEnum converts the string representation of the type of command to an equivalent 
     *        enum representation
     * 
     * @param aType the type of the command in string representation
     * 
     * @return the type of command in CommandType enum representation
     */
    CommandType toEnum(const std::string& aType); 

    /**
     * @brief toString converts the enum representation of a command type to a string
     * 
     * @param aCmdType the type of command in CommandType enum representation
     * 
     * @return a std::string representation of the type of command
     */
    std::string toString(CommandType aCmdType); 

    /**
     * @brief setNewActiveState changes the active state of the state machine based on the command received
     * 
     * @param aNewState an enum representation of the new state to transition the state machine to
     */
    void setNewActiveState(StateMachine::States aNewState); 

    /**
     * @brief convertToEigen converts the interface message representation of a vector to an Eigen representation
     * 
     * @param aVectorToConvert the vector of data in the interface message representation 
     * 
     * @return the equivalent vector in Eigen representation 
     */
    Eigen::Vector3d convertToEigen(abv_msgs::msg::AbvVec3 aVectorToConvert);

private: 
    std::shared_ptr<StateMachine> mStateMachine;
    std::shared_ptr<GoalManager> mGoalManager;

};
#endif // COMMANDHANDLER_H
