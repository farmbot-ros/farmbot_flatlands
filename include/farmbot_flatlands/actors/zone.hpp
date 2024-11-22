#ifndef ZONE_HPP
#define ZONE_HPP

#include <vector>
#include <memory>
#include <string>

// Include ROS geometry messages
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon.hpp"

// For the Robot state
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "farmbot_flatlands/actors/actor.hpp"

// Other necessary includes
#include <rclcpp/rclcpp.hpp>

namespace sim {
    namespace actor{
        class Zone : Actor {
            private:
                std::string name_;
                geometry_msgs::msg::Polygon area_;
            public:
                Zone(const std::string& name, const geometry_msgs::msg::Polygon& area);
                bool contains(const geometry_msgs::msg::Point& point) const override;
                ActorType get_type() const override { return ActorType::ZONE; }
                std::string get_name() const { return name_; }
            };
        inline Zone::Zone(const std::string& name, const geometry_msgs::msg::Polygon& area)
            : name_(name), area_(area) {}
        inline bool Zone::contains(const geometry_msgs::msg::Point& point) const {
            return false; // Placeholder
        }

    }// namespace actor
}// namespace sim

#endif // OBSTACLE_HPP
