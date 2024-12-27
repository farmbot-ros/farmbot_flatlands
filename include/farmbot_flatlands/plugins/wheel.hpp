#ifndef WHEEL_PLUGIN_HPP
#define WHEEL_PLUGIN_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <string>
#include <memory>

#include "farmbot_flatlands/plugins/plugin.hpp"

namespace sim {
    namespace plugins {
        class WheelPlugin :Plugin {
            private:
                rclcpp::Node::SharedPtr node_;  // ROS 2 node handle
                std::string topic_;              // Base topic name for gyroscope data
                nav_msgs::msg::Odometry odom_;   // Odometry data

                rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_pub_;  // Publisher for wheel data

            public:
                WheelPlugin() = default;
                WheelPlugin(rclcpp::Node::SharedPtr node, const std::string & topic_name, const nav_msgs::msg::Odometry & odom) : node_(node), topic_(topic_name) {
                    RCLCPP_INFO(node_->get_logger(), "Wheel Odom Plugin initialized on topic: %s", topic_.c_str());
                    // Initialize publisher
                    wheel_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(topic_, 10);

                }

                void init(const std::vector<std::any>& args) override { return; }

                void tick(const rclcpp::Time & current_time, const nav_msgs::msg::Odometry & odom) override {
                    wheel_pub_->publish(odom);
                }

        };
    } // namespace plugins
} // namespace sim

#endif // WHEEL_PLUGIN_HPP
