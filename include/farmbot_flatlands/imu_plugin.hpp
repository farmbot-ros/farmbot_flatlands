#ifndef IMU_PLUGIN_HPP
#define IMU_PLUGIN_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"

#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <string>
#include <memory>
#include <cmath>

namespace sim {
    namespace plugins {
        class IMUPlugin {
            private:
                rclcpp::Node::SharedPtr node_;
                std::string topic_;
                sensor_msgs::msg::Imu imu_msg;
                nav_msgs::msg::Odometry odom_;
                rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

                // Previous velocities for acceleration calculation
                geometry_msgs::msg::Twist prev_twist_;
                rclcpp::Time last_time_;

            public:
                IMUPlugin() = default;
                IMUPlugin(rclcpp::Node::SharedPtr node, std::string topic, const nav_msgs::msg::Odometry & odom):
                    node_(node),
                    topic_(topic),
                    prev_twist_(odom.twist.twist),
                    last_time_(node_->now())
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IMU Plugin initialized");
                    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic_, 10);
                }

                void tick(const rclcpp::Time & current_time, nav_msgs::msg::Odometry & odom) {
                    odom_ = odom;
                    // Calculate time difference
                    double delta_t = (current_time - last_time_).seconds();
                    if (delta_t <= 0.0) {
                        delta_t = 1.0 / 10.0; // Default to 10 Hz if invalid
                    }

                    // Calculate linear acceleration
                    double accel_x = (odom_.twist.twist.linear.x - prev_twist_.linear.x) / delta_t;
                    double accel_y = (odom_.twist.twist.linear.y - prev_twist_.linear.y) / delta_t;
                    double accel_z = (odom_.twist.twist.linear.z - prev_twist_.linear.z) / delta_t;

                    // Prepare IMU message
                    imu_msg.header.stamp = current_time;
                    imu_msg.header.frame_id = "imu_link";

                    // Orientation
                    imu_msg.orientation = odom_.pose.pose.orientation;

                    // Angular velocity
                    imu_msg.angular_velocity = odom_.twist.twist.angular;

                    // Linear acceleration
                    imu_msg.linear_acceleration.x = accel_x;
                    imu_msg.linear_acceleration.y = accel_y;
                    imu_msg.linear_acceleration.z = accel_z;

                    // Set covariances to zero or appropriate values
                    for (int i = 0; i < 9; ++i) {
                        imu_msg.orientation_covariance[i] = 0.0;
                        imu_msg.angular_velocity_covariance[i] = 0.0;
                        imu_msg.linear_acceleration_covariance[i] = 0.0;
                    }

                    // Update previous twist and time
                    prev_twist_ = odom_.twist.twist;
                    last_time_ = current_time;

                    // RCLCPP_INFO(node_->get_logger(), "IMU - Accel: [%f, %f, %f]", accel_x, accel_y, accel_z);
                    imu_pub_->publish(imu_msg);
                }
        };
    } // namespace plugins
} // namespace mobile_robot_simulator

#endif // IMU_PLUGIN_HPP
