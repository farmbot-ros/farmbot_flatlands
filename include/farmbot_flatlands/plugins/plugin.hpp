#ifndef PLUGIN_HPP
#define PLUGIN_HPP

#include <string>
#include <tuple>
#include <cmath>
#include <any>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace sim {
    class Plugin {
        public:
            virtual ~Plugin() = default;
            virtual void init(const std::vector<std::any>& args) = 0;
            virtual void tick(const rclcpp::Time &current_time, const nav_msgs::msg::Odometry & odom) = 0;
    };
}

#endif
