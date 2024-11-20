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

namespace sim {
    namespace plugins {
        class CompassPlugin {
            private:
                rclcpp::Node::SharedPtr node_;  // ROS 2 node handle
                std::string topic_;              // Topic name for compass data
                std_msgs::msg::Float32 compass_msg_;  // Compass message
                rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr compass_pub_;  // Publisher

            public:
                CompassPlugin() = default;
                CompassPlugin(rclcpp::Node::SharedPtr node, const std::string & topic_name, const nav_msgs::msg::Odometry & odom) : node_(node), topic_(topic_name){
                    RCLCPP_INFO(node_->get_logger(), "Compass Plugin initialized on topic: %s", topic_.c_str());
                    // Initialize publisher
                    compass_pub_ = node_->create_publisher<std_msgs::msg::Float32>(topic_, 10);

                    // Initialize compass_msg_
                    compass_msg_.data = 0.0; // Initial heading
                }

                ~CompassPlugin() = default;
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
                    compass_msg_.data = yaw; // Heading in radians
                    // Publish the compass data
                    compass_pub_->publish(compass_msg_);
                    // Optional: Debug logging
                    // RCLCPP_DEBUG(node_->get_logger(), "Compass - Heading (yaw): %.2f radians", yaw);
                }
        };
    } // namespace plugins
} // namespace sim

#endif // COMPASS_PLUGIN_HPP
