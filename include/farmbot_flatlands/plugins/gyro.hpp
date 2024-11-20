#ifndef GYRO_PLUGIN_HPP
#define GYRO_PLUGIN_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <string>
#include <memory>

namespace sim {
    namespace plugins {
        /**
         * @brief GyroPlugin class for publishing orientation data.
         *
         * This plugin extracts the robot's orientation from the odometry
         * and publishes it as a separate ROS 2 topic.
         */
        class GyroPlugin {
            private:
                rclcpp::Node::SharedPtr node_;  // ROS 2 node handle
                std::string topic_;              // Base topic name for gyroscope data
                geometry_msgs::msg::QuaternionStamped gyro_msg_;  // Orientation message
                rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr gyro_pub_;  // Publisher

            public:
                /**
                 * @brief Default constructor.
                 */
                GyroPlugin() = default;

                /**
                 * @brief Parameterized constructor.
                 *
                 * @param node Shared pointer to the ROS 2 node.
                 * @param topic_name Base name of the topic to publish gyroscope data.
                 * @param odom Initial odometry data (unused but included for consistency).
                 */
                GyroPlugin(rclcpp::Node::SharedPtr node, const std::string & topic_name, const nav_msgs::msg::Odometry & odom)
                    : node_(node), topic_(topic_name)
                {
                    RCLCPP_INFO(node_->get_logger(), "Gyro Plugin initialized on topic: %s/gyro", topic_.c_str());
                    // Initialize publisher
                    gyro_pub_ = node_->create_publisher<geometry_msgs::msg::QuaternionStamped>(topic_ + "/gyro", 10);

                    // Initialize gyro_msg_
                    gyro_msg_.header.frame_id = "gyro_link";  // Ensure this matches your TF setup
                    gyro_msg_.quaternion.x = 0.0;
                    gyro_msg_.quaternion.y = 0.0;
                    gyro_msg_.quaternion.z = 0.0;
                    gyro_msg_.quaternion.w = 1.0;
                }

                /**
                 * @brief Destructor.
                 */
                ~GyroPlugin() = default;

                /**
                 * @brief Updates and publishes orientation data.
                 *
                 * @param current_time Current simulated time.
                 * @param odom Current odometry data.
                 * @param new_data Flag indicating if new odometry data is available.
                 */
                void tick(const rclcpp::Time & current_time, const nav_msgs::msg::Odometry & odom, bool new_data = true) {
                    if (!new_data) {
                        RCLCPP_WARN(node_->get_logger(), "GyroPlugin tick called without new odometry data.");
                        return;
                    }

                    // Extract orientation from odometry
                    geometry_msgs::msg::Quaternion orientation = odom.pose.pose.orientation;

                    // Populate gyro_msg_
                    gyro_msg_.header.stamp = current_time;
                    gyro_msg_.quaternion = orientation;

                    // Publish the orientation data
                    gyro_pub_->publish(gyro_msg_);

                    // Optional: Debug logging
                    // RCLCPP_DEBUG(node_->get_logger(), "Gyro - Orientation: [%.2f, %.2f, %.2f, %.2f]",
                    //              orientation.x, orientation.y, orientation.z, orientation.w);
                }
        };
    } // namespace plugins
} // namespace sim

#endif // GYRO_PLUGIN_HPP
