#ifndef TOPICCONVERSIONS_HPP
#define TOPICCONVERSIONS_HPP


// incoming types
#include "abv_msgs/msg/abv_state.hpp"
#include "abv_msgs/msg/abv_controller_status.hpp"
#include "abv_msgs/msg/abv_controller_command.hpp"
#include "abv_msgs/msg/abv_thruster_status.hpp"
#include "abv_msgs/msg/abv_path.hpp"

// outgoing types
#include <QVector>
#include <QPointF>

namespace conversions
{
    inline QVector<double> 
    navigationPositionConvertor(const abv_msgs::msg::AbvState& msg)
    {
        return QVector<double>{
            msg.position.x,
            msg.position.y,
            msg.position.yaw
        }; 
    }

    inline QVector<double>
    navigationStateConvertor(const abv_msgs::msg::AbvState& msg)
    {
        return QVector<double>{
            msg.position.x,
            msg.position.y,
            msg.position.yaw,
            msg.valid ? 1.0 : 0.0
        };
    }

    inline QVector<double>
    navigationVelocityConvertor(const abv_msgs::msg::AbvState& msg)
    {
        return QVector<double> {
            msg.velocity.x, 
            msg.velocity.y, 
            msg.velocity.yaw
        };
    }

    inline QVector<double>
    thrusterStatusConvertor(const abv_msgs::msg::AbvThrusterStatus& msg)
    {
        return QVector<double> {
            msg.fx,
            msg.fy,
            msg.tz
        };
    }

    inline QVector<QPointF>
    pathConvertor(const abv_msgs::msg::AbvPath& msg)
    {
        QVector<QPointF> path;
        path.reserve(static_cast<int>(msg.waypoints.size()));
        for (const auto& wp : msg.waypoints)
        {
            path.push_back(QPointF(wp.x, wp.y));
        }
        return path;
    }

} // namespace topic
#endif