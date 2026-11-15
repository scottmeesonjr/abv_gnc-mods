
#include <array>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string> 
#include <vector> 
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "abv_commander/InputHandler.h"
#include "abv_common/RosTopicManager.h"
#include "abv_commander/CommanderComms.h"
#include "abv_msgs/msg/dock_command.hpp"
#include "std_msgs/msg/string.hpp"

InputHandler::InputHandler()
{
    mPackagePath = ament_index_cpp::get_package_share_directory("abv_commander");
    std::cout << "Package path: " << mPackagePath << std::endl; 
}

InputHandler::~InputHandler()
{

}

bool InputHandler::promptNumber(const std::string& aPrompt, double aDefault, double& aOut)
{
    std::string text;
    std::cout << GREEN << aPrompt;
    std::getline(std::cin, text);

    std::size_t start = text.find_first_not_of(" \t");
    if (start == std::string::npos)
    {
        aOut = aDefault;
        return true;
    }
    try
    {
        std::size_t used = 0;
        aOut = std::stod(text.substr(start), &used);
        if (text.find_first_not_of(" \t", start + used) != std::string::npos)
        {
            throw std::invalid_argument("trailing characters");
        }
        return true;
    }
    catch (const std::exception&)
    {
        std::cout << RED << "Not a number: '" << text << "'" << RESET << std::endl;
        return false;
    }
}

void InputHandler::sendDockRun()
{
    double radial = 0.0, angleDeg = 0.0, speed = 0.0, duration = -1.0;
    if (!promptNumber("Radial offset (m) [0]: ", 0.0, radial)) return;
    if (!promptNumber("Angular offset (deg) [0]: ", 0.0, angleDeg)) return;
    if (!promptNumber("Approach speed (m/s) [config]: ", 0.0, speed)) return;
    if (!promptNumber("Duration (s) [no limit]: ", -1.0, duration)) return;

    std::string runName;
    std::cout << GREEN << "Run name for the log file [none]: ";
    std::getline(std::cin, runName);

    if (std::abs(radial) > 0.10 || std::abs(angleDeg) > 4.0)
    {
        std::cout << CYAN << "Note: beyond the SYS.07/SYS.08 limits (0.10 m, 4 deg)" << RESET << std::endl;
    }
    if (speed != 0.0 && (speed < 0.05 || speed > 0.10))
    {
        std::cout << CYAN << "Note: outside the SYS.10 band (0.05-0.10 m/s)" << RESET << std::endl;
    }

    // goal x = radial offset (m), goal y = approach speed (m/s, 0 = config),
    // goal yaw = angular offset (rad). See CyclicDockGenerator.h.
    abv_msgs::msg::AbvVec3 goal;
    goal.set__x(radial);
    goal.set__y(speed);
    goal.set__yaw(angleDeg * M_PI / 180.0);

    abv_msgs::msg::AbvState goalState;
    goalState.set__position(goal);

    // name first, so the logger has it before the run's first ARM
    std_msgs::msg::String name;
    name.data = runName;
    RosTopicManager::getInstance()->publishMessage<std_msgs::msg::String>("abv/test/run_name", name);

    abv_msgs::msg::AbvGuidanceCommand cmd;
    cmd.set__type("dock");
    cmd.set__duration(duration);
    cmd.set__goal_state(goalState);

    RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvGuidanceCommand>("abv/guidance/command", cmd);
    std::cout << "Sent dock run: radial " << radial << " m, angular " << angleDeg << " deg, speed "
              << (speed == 0.0 ? std::string("config") : std::to_string(speed) + " m/s") << std::endl;
}

void InputHandler::sendDockCommand(std::uint8_t aCode, const std::string& aName)
{
    abv_msgs::msg::DockCommand cmd;
    cmd.stamp = RosTopicManager::getInstance()->get_clock()->now();
    cmd.command = aCode;
    RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::DockCommand>("abv/dock/cmd", cmd);
    std::cout << "Sent mechanism command: " << aName << std::endl;
}

void InputHandler::handle(const std::string& anInput)
{
    if (anInput.empty())
    {
        return;
    }
    else if ("pose" == anInput)
    {
        std::array<double, 3> vehPose;
        std::string vehPoseStr;

        std::cout << "Enter goal pose (x y yaw): ";
        std::getline(std::cin, vehPoseStr);

        std::istringstream iss(vehPoseStr);
        for (int i = 0; i < 3; ++i)
        {
            if (!(iss >> vehPose[i]))
            {
                std::cerr << "Invalid input. Please enter three space-separated numbers." << std::endl;
                return;
            }
        }

        abv_msgs::msg::AbvVec3 data; 
        data.set__x(vehPose[0]); 
        data.set__y(vehPose[1]); 
        data.set__yaw(vehPose[2]); 

        abv_msgs::msg::AbvControllerCommand cmd; 
        cmd.set__data(data); 
        cmd.set__type("pose");
        
        RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvControllerCommand>("abv/controller/command", cmd); 
        std::cout << "Published goal waypoint (x y yaw): " << vehPose[0] << ", " << vehPose[1] << ", " << vehPose[2] << std::endl; 
    }
    else if ("vel" == anInput)
    {
        std::array<double, 3> vehPose;
        std::string vehPoseStr;

        std::cout << "Enter goal velocity (xd yd yawd): ";
        std::getline(std::cin, vehPoseStr);

        std::istringstream iss(vehPoseStr);
        for (int i = 0; i < 3; ++i)
        {
            if (!(iss >> vehPose[i]))
            {
                std::cerr << "Invalid input. Please enter three space-separated numbers." << std::endl;
                return;
            }
        }

        abv_msgs::msg::AbvVec3 data; 
        data.set__x(vehPose[0]); 
        data.set__y(vehPose[1]); 
        data.set__yaw(vehPose[2]); 

        abv_msgs::msg::AbvControllerCommand cmd; 
        cmd.set__data(data); 
        cmd.set__type("velocity");
        
        RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvControllerCommand>("abv/controller/command", cmd); 
        std::cout << "Published goal velocity waypoint (xd yd yawd): " << vehPose[0] << ", " << vehPose[1] << ", " << vehPose[2] << std::endl;
    }
    else if ("stop" == anInput)
    {
        abv_msgs::msg::AbvVec3 data; 
        data.set__x(0); 
        data.set__y(0); 
        data.set__yaw(0); 

        abv_msgs::msg::AbvControllerCommand cmd; 
        cmd.set__data(data); 
        cmd.set__type("STOP");
        
        RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvControllerCommand>("abv/controller/command", cmd); 
    }
    else if ("path" == anInput)
    {
        std::string trajType;

        std::cout << GREEN << "Type (file, line, avoid, dock): "; 
        std::getline(std::cin, trajType);

        if ("dock" == trajType || "Dock" == trajType)
        {
            sendDockRun();
            return;
        }

        double dur = -1.0;
        if (!promptNumber("Duration (s) [no limit]: ", -1.0, dur)) return;

        abv_msgs::msg::AbvGuidanceCommand cmd; 
        cmd.set__type(trajType); 
        cmd.set__duration(dur); 

        RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvGuidanceCommand>("abv/guidance/command", cmd); 
    }
    else if ("dock" == anInput)
    {
        sendDockRun();
    }
    else if ("condition" == anInput)
    {
        std::string text;
        std::cout << GREEN << "Test condition note (copied into every docking log row): ";
        std::getline(std::cin, text);

        std_msgs::msg::String msg;
        msg.data = text;
        RosTopicManager::getInstance()->publishMessage<std_msgs::msg::String>("abv/test/condition", msg);
        std::cout << "Condition set: " << text << std::endl;
    }
    else if ("arm" == anInput)    { sendDockCommand(abv_msgs::msg::DockCommand::CMD_ARM, "ARM"); }
    else if ("disarm" == anInput) { sendDockCommand(abv_msgs::msg::DockCommand::CMD_DISARM, "DISARM"); }
    else if ("lock" == anInput)   { sendDockCommand(abv_msgs::msg::DockCommand::CMD_LOCK, "LOCK"); }
    else if ("unlock" == anInput) { sendDockCommand(abv_msgs::msg::DockCommand::CMD_UNLOCK, "UNLOCK"); }
    else if (anInput == "help" || anInput == "--help" || anInput == "-h") 
    {
        std::cout << R"(
        ABV Commander Tool
        -----------------------------
        Interact with the air-bearing vehicle by sending pose or velocity waypoints. 
        
        Available Commands:
        
        pose:       command the abv to a desired x, y, yaw pose 
        vel:        command the abv to a desired x, y, yaw velocity 
        stop:       switch the thrusters off (the vehicle will coast)
        path:       command the abv to follow a path sent by the guidance node 
        dock:       start a docking run (asks for offsets, approach speed, duration)
        condition:  set a test-condition note for the docking log
        arm, disarm, lock, unlock:
                    send a command straight to the docking mechanism
        
        Blank answers use the default shown in [brackets].
        Type 'help' or '--help' to show this message again.
        )" << std::endl;
    }       
    else{
        std::cout << RED << "Unsupported command! Type 'help' for the list." << RESET << std::endl; 
    }
}
