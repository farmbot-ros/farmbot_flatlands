#ifndef IMU_PLUGIN_HPP
#define IMU_PLUGIN_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"

#include <string>
#include <memory>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace sim {
    namespace plugins {
        class IMUPlugin {
            private:
                rclcpp::Node::SharedPtr node_;
                std::string topic_;
                sensor_msgs::msg::Imu imu_msg_;
                nav_msgs::msg::Odometry odom_;
                rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

                // Previous velocities for acceleration calculation
                geometry_msgs::msg::Twist prev_twist_;
                rclcpp::Time last_time_;

            public:
                IMUPlugin() = default;

                IMUPlugin(rclcpp::Node::SharedPtr node, const std::string & topic, const nav_msgs::msg::Odometry & odom)
                    : node_(node),
                      topic_(topic),
                      prev_twist_(odom.twist.twist),
                      last_time_(node_->now())
                {
                    RCLCPP_INFO(node_->get_logger(), "IMU Plugin initialized on topic: %s", topic_.c_str());
                    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic_ + "/data", 10);

                    // Initialize IMU message covariances to default values (optional)
                    // Here, we're setting them to unknown (set to -1)
                    for (int i = 0; i < 9; ++i) {
                        imu_msg_.orientation_covariance[i] = -1;
                        imu_msg_.angular_velocity_covariance[i] = -1;
                        imu_msg_.linear_acceleration_covariance[i] = -1;
                    }
                }

                /**
                 * @brief Updates the IMU data based on the latest odometry information.
                 *
                 * @param current_time The current simulated time.
                 * @param odom The latest odometry data.
                 * @param new_data Flag indicating if new odometry data is available.
                 */
                void tick(const rclcpp::Time & current_time, const nav_msgs::msg::Odometry & odom, bool new_data = true) {
                    odom_ = odom;
                    // Calculate time difference
                    double delta_t = (current_time - last_time_).seconds();
                    if (delta_t <= 0.0) {
                        RCLCPP_WARN(node_->get_logger(), "Non-positive delta_t encountered in IMUPlugin. Using default value.");
                        delta_t = 1.0 / 10.0; // Default to 10 Hz if invalid
                    }

                    // Calculate linear acceleration (change in velocity over time)
                    double accel_x = (odom_.twist.twist.linear.x - prev_twist_.linear.x) / delta_t;
                    double accel_y = (odom_.twist.twist.linear.y - prev_twist_.linear.y) / delta_t;
                    double accel_z = (odom_.twist.twist.linear.z - prev_twist_.linear.z) / delta_t;

                    // Prepare IMU message
                    imu_msg_.header.stamp = current_time;
                    imu_msg_.header.frame_id = "imu_link"; // Ensure this matches your TF setup

                    // Orientation: directly use odometry's orientation
                    imu_msg_.orientation = odom_.pose.pose.orientation;

                    // Angular velocity: directly use odometry's angular velocity
                    imu_msg_.angular_velocity = odom_.twist.twist.angular;

                    // Linear acceleration: use calculated accelerations
                    imu_msg_.linear_acceleration.x = accel_x;
                    imu_msg_.linear_acceleration.y = accel_y;
                    imu_msg_.linear_acceleration.z = accel_z;

                    // Optionally, add gravity compensation if necessary
                    // Example: imu_msg_.linear_acceleration.z += 9.81;

                    // Update previous twist and time for next acceleration calculation
                    prev_twist_ = odom_.twist.twist;
                    last_time_ = current_time;

                    // Debug information (optional)
                    // RCLCPP_DEBUG(node_->get_logger(), "IMU - Accel: [%f, %f, %f]", accel_x, accel_y, accel_z);

                    // Publish the IMU message
                    imu_pub_->publish(imu_msg_);
                }
        };
    } // namespace plugins
} // namespace sim

#endif // IMU_PLUGIN_HPP
