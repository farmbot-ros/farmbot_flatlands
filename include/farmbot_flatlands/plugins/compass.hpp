#ifndef COMPASS_PLUGIN_HPP
#define COMPASS_PLUGIN_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <string>
#include <memory>
#include <mutex>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "farmbot_flatlands/plugins/plugin.hpp"

double toRadians(double degrees) {
    auto rads = std::fmod(degrees * M_PI / 180.0, 360.0);
    return rads;
}

double toDegrees(double radians) {
    auto degs = std::fmod(radians * 180.0 / M_PI, 360.0);
    return degs;
}

namespace sim {
    namespace plugins {
        class CompassPlugin : Plugin {
            private:
                rclcpp::Node::SharedPtr node_;  // ROS 2 node handle
                std::string topic_;              // Topic name for compass data
                std_msgs::msg::Float32 compass_msg_;  // Compass message
                rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rad_pub;  // Publisher
                rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr deg_pub;  // Publisher

            public:
                CompassPlugin() = default;
                CompassPlugin(rclcpp::Node::SharedPtr node, const std::string & topic_name, const nav_msgs::msg::Odometry & odom) : node_(node), topic_(topic_name){
                    RCLCPP_INFO(node_->get_logger(), "Compass Plugin initialized on topic: %s", topic_name.c_str());
                    // Initialize publisher
                    rad_pub = node_->create_publisher<std_msgs::msg::Float32>(topic_ + "/radians", 10);
                    deg_pub = node_->create_publisher<std_msgs::msg::Float32>(topic_ + "/degrees", 10);

                    // Initialize compass_msg_
                    compass_msg_.data = 0.0; // Initial heading
                }

                void tick2(const rclcpp::Time &current_time) override { return; }

                void tick(const rclcpp::Time & current_time, const nav_msgs::msg::Odometry & odom, bool new_data = true) {
                    // if (!new_data) {
                    //     RCLCPP_WARN(node_->get_logger(), "CompassPlugin tick called without new odometry data.");
                    //     return;
                    // }
                    // Extract orientation from odometry
                    tf2::Quaternion orientation;
                    tf2::fromMsg(odom.pose.pose.orientation, orientation);
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
                    // Publish the compass data
                    compass_msg_.data = yaw; // Heading in radians
                    rad_pub->publish(compass_msg_);
                    compass_msg_.data = toDegrees(yaw); // Heading in degrees
                    deg_pub->publish(compass_msg_);
                    // Optional: Debug logging
                    // RCLCPP_DEBUG(node_->get_logger(), "Compass - Heading (yaw): %.2f radians", yaw);
                }
        };
    } // namespace plugins
} // namespace sim

#endif // COMPASS_PLUGIN_HPP
