#ifndef TOPICCONVERSIONS_HPP
#define TOPICCONVERSIONS_HPP


// incoming types
#include "abv_msgs/msg/abv_state.hpp"
#include "abv_msgs/msg/abv_controller_status.hpp"
#include "abv_msgs/msg/abv_controller_command.hpp"
#include "abv_msgs/msg/abv_thruster_status.hpp"
#include "abv_msgs/msg/abv_path.hpp"
#include "abv_msgs/msg/dock_status.hpp"
#include "abv_msgs/msg/nav_alert.hpp"

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

    inline QVector<double>
    dockStatusConvertor(const abv_msgs::msg::DockStatus& msg)
    {
        return QVector<double> {
            static_cast<double>(msg.state),
            msg.locked ? 1.0 : 0.0,
            msg.link_ok ? 1.0 : 0.0,
            msg.estop ? 1.0 : 0.0,
            static_cast<double>(msg.cycle_count),
            static_cast<double>(msg.fault_code)
        };
    }

    inline QVector<double>
    navAlertConvertor(const abv_msgs::msg::NavAlert& msg)
    {
        return QVector<double> {
            msg.speed,
            msg.closing_velocity,
            msg.range,
            msg.alert ? 1.0 : 0.0
        };
    }

} // namespace topic
#endif