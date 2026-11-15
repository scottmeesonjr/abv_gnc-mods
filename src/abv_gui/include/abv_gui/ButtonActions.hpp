#ifndef BUTTONACTIONS_HPP
#define BUTTONACTIONS_HPP

#include "abv_common/RosTopicManager.h"
#include "abv_msgs/msg/abv_controller_command.hpp"
#include "abv_msgs/msg/abv_guidance_command.hpp"

namespace btn
{
    namespace action
    {
        inline void stop()
        {
            abv_msgs::msg::AbvControllerCommand cmd; 
            cmd.set__type("STOP"); 

            RosTopicManager::getInstance()->publishMessage("abv/controller/command", cmd); 

            // also end any path guidance is running, or it would keep sending
            // waypoints and restart the vehicle
            abv_msgs::msg::AbvGuidanceCommand gcmd;
            gcmd.set__type("stop");
            RosTopicManager::getInstance()->publishMessage("abv/guidance/command", gcmd);
        }

        // Stop switches the thrusters off, so on a frictionless table the
        // vehicle keeps coasting. Brake ends the path and holds zero velocity.
        inline void brake()
        {
            abv_msgs::msg::AbvGuidanceCommand gcmd;
            gcmd.set__type("stop");
            RosTopicManager::getInstance()->publishMessage("abv/guidance/command", gcmd);

            abv_msgs::msg::AbvControllerCommand cmd;
            cmd.set__type("velocity");
            cmd.set__is_global(true);
            RosTopicManager::getInstance()->publishMessage("abv/controller/command", cmd);
        }

    } // namespace action
    
} // namespace btn
#endif 