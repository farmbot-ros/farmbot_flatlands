#ifndef SIM_HPP
#define SIM_HPP

#include "rclcpp/rclcpp.hpp"

// Message Types
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

// Plugins (Header-Only)
#include "farmbot_flatlands/plugins/gps.hpp"
#include "farmbot_flatlands/plugins/imu.hpp"

// TF2 Headers
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Standard Libraries
#include <curl/curl.h>
#include <ios>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <cmath>
#include <tuple>
#include <future> // For std::async
#include <algorithm> // For std::clamp

using namespace std::chrono_literals;

namespace sim {
    class SIM : public rclcpp::Node {
        private:
            // Robot state
            sensor_msgs::msg::NavSatFix fix_;
            nav_msgs::msg::Odometry odom_;
            std_msgs::msg::Float32 theta_;

            // Quaternion
            tf2::Quaternion quat;

            // Parameters
            double publish_rate_;
            std::string vel_topic_;
            std::string fix_topic_;
            std::string imu_topic_;

            double latitude_;
            double longitude_;
            double altitude_;
            double heading_;

            // Time tracking
            rclcpp::Time last_update_;
            rclcpp::Time simulated_time_;

            // ROS Interfaces
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
            rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

            // Timer
            rclcpp::TimerBase::SharedPtr loop_timer_;
            bool new_message_ = false;

            // GPS Plugin
            plugins::GPSPlugin gps_plugin_;
            // IMU Plugin
            plugins::IMUPlugin imu_plugin_;

            // Velocity Control Variables
            geometry_msgs::msg::Twist current_twist_;
            geometry_msgs::msg::Twist target_twist_;
            double max_linear_accel_;   // meters per second squared
            double max_angular_accel_;  // radians per second squared

            // Thread synchronization (optional, if needed)
            // std::mutex odom_mutex_;

        public:
            SIM(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("simulator", options) {
                // Declare and get parameters
                this->declare_parameter<double>("publish_rate", 10.0);
                this->get_parameter("publish_rate", publish_rate_);
                this->declare_parameter<std::string>("velocity_topic", "cmd_vel");
                this->get_parameter("velocity_topic", vel_topic_);
                this->declare_parameter<std::string>("imu_topic", "imu");
                this->get_parameter("imu_topic", imu_topic_);
                this->declare_parameter<std::string>("gnss_topic", "gnss");
                this->get_parameter("gnss_topic", fix_topic_);
                this->declare_parameter<double>("latitude", 0.0);
                this->get_parameter("latitude", latitude_);
                this->declare_parameter<double>("longitude", 0.0);
                this->get_parameter("longitude", longitude_);
                this->declare_parameter<double>("altitude", 0.0);
                this->get_parameter("altitude", altitude_);
                this->declare_parameter<double>("heading", 0.0);
                this->get_parameter("heading", heading_);

                // Initialize odometry message
                odom_.header.frame_id = "world";
                theta_.data = heading_ * M_PI / 180.0;
                quat.setRPY(0, 0, theta_.data);
                odom_.pose.pose.orientation = tf2::toMsg(quat);

                // Initialize fix message
                fix_.header.frame_id = "world";
                fix_.latitude = latitude_;
                fix_.longitude = longitude_;
                fix_.altitude = altitude_;

                // Subscriber for velocity commands
                vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                    vel_topic_, 10,
                    std::bind(&SIM::vel_callback, this, std::placeholders::_1)
                );

                // Publisher for simulated clock
                clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

                // Initialize simulated time
                simulated_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
                last_update_ = simulated_time_;

                // Initialize current and target velocities to zero
                current_twist_.linear.x = 0.0;
                current_twist_.linear.y = 0.0;
                current_twist_.linear.z = 0.0;
                current_twist_.angular.x = 0.0;
                current_twist_.angular.y = 0.0;
                current_twist_.angular.z = 0.0;

                target_twist_ = current_twist_; // Initially, target is same as current

                // Initialize maximum accelerations (tunable parameters)
                this->declare_parameter<double>("max_linear_accel", 0.5);   // m/s²
                this->get_parameter("max_linear_accel", max_linear_accel_);
                this->declare_parameter<double>("max_angular_accel", 0.5);  // rad/s²
                this->get_parameter("max_angular_accel", max_angular_accel_);

                RCLCPP_INFO(this->get_logger(), "SIM initialized.");
            }

            ~SIM(){
                if (loop_timer_) {
                    loop_timer_->cancel();
                }
                RCLCPP_INFO(this->get_logger(), "Simulator stopped.");
            }

            void start(){
                // Create GPS Plugin
                gps_plugin_ = plugins::GPSPlugin(this->shared_from_this(), fix_topic_, fix_);
                // Create IMU Plugin
                imu_plugin_ = plugins::IMUPlugin(this->shared_from_this(), imu_topic_, odom_);
                // Start the simulator
                loop_timer_ = this->create_wall_timer(
                    std::chrono::duration<double>(1.0 / publish_rate_),
                    std::bind(&SIM::update_loop, this)
                );
            }

        private:
            void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
                target_twist_ = *msg; // Set target velocities based on incoming Twist message
                new_message_ = true;
            }

            void update_loop(){
                // Increment simulated time
                simulated_time_ += rclcpp::Duration::from_seconds(1.0 / publish_rate_);

                // Compute time difference
                rclcpp::Duration dt = simulated_time_ - last_update_;
                double delta_t = dt.seconds();
                if (delta_t <= 0.0) {
                    delta_t = 1.0 / publish_rate_;
                }

                // Calculate the required acceleration to reach target velocities
                // Linear Velocities
                double accel_x = (target_twist_.linear.x - current_twist_.linear.x) / delta_t;
                double accel_y = (target_twist_.linear.y - current_twist_.linear.y) / delta_t;
                double accel_z = (target_twist_.linear.z - current_twist_.linear.z) / delta_t;

                // Clamp the accelerations to the maximum limits
                accel_x = std::clamp(accel_x, -max_linear_accel_, max_linear_accel_);
                accel_y = std::clamp(accel_y, -max_linear_accel_, max_linear_accel_);
                accel_z = std::clamp(accel_z, -max_linear_accel_, max_linear_accel_);

                // Update current linear velocities
                current_twist_.linear.x += accel_x * delta_t;
                current_twist_.linear.y += accel_y * delta_t;
                current_twist_.linear.z += accel_z * delta_t;

                // Angular Velocities
                double angular_accel_x = (target_twist_.angular.x - current_twist_.angular.x) / delta_t;
                double angular_accel_y = (target_twist_.angular.y - current_twist_.angular.y) / delta_t;
                double angular_accel_z = (target_twist_.angular.z - current_twist_.angular.z) / delta_t;

                // Clamp the angular accelerations
                angular_accel_x = std::clamp(angular_accel_x, -max_angular_accel_, max_angular_accel_);
                angular_accel_y = std::clamp(angular_accel_y, -max_angular_accel_, max_angular_accel_);
                angular_accel_z = std::clamp(angular_accel_z, -max_angular_accel_, max_angular_accel_);

                // Update current angular velocities
                current_twist_.angular.x += angular_accel_x * delta_t;
                current_twist_.angular.y += angular_accel_y * delta_t;
                current_twist_.angular.z += angular_accel_z * delta_t;

                // Assign the smoothly updated velocities to odometry
                odom_.twist.twist = current_twist_;

                // Convert current orientation from quaternion to tf2 Quaternion
                tf2::Quaternion current_orientation;
                tf2::fromMsg(odom_.pose.pose.orientation, current_orientation);

                // Extract linear velocity from odometry (robot frame)
                tf2::Vector3 linear_vel(current_twist_.linear.x,
                                        current_twist_.linear.y,
                                        current_twist_.linear.z);

                // Transform linear velocity to world frame using current orientation
                tf2::Vector3 linear_vel_world = tf2::quatRotate(current_orientation, linear_vel);

                // Compute delta position in world frame
                tf2::Vector3 delta_pos = linear_vel_world * delta_t;

                // Update position
                odom_.pose.pose.position.x += delta_pos.x();
                odom_.pose.pose.position.y += delta_pos.y();
                odom_.pose.pose.position.z += delta_pos.z();

                // Extract angular velocity from odometry (robot frame)
                double omega_x = current_twist_.angular.x;
                double omega_y = current_twist_.angular.y;
                double omega_z = current_twist_.angular.z;

                // Create angular velocity vector
                tf2::Vector3 angular_vel(omega_x, omega_y, omega_z);
                double angular_speed = angular_vel.length();

                // Compute the change in orientation as a quaternion
                tf2::Quaternion delta_q;
                if (angular_speed > 1e-6) { // Avoid division by zero
                    tf2::Vector3 rotation_axis = angular_vel.normalized();
                    double rotation_angle = angular_speed * delta_t;
                    delta_q.setRotation(rotation_axis, rotation_angle);
                } else {
                    delta_q = tf2::Quaternion(0, 0, 0, 1); // Identity quaternion
                }

                // Update the current orientation
                current_orientation = current_orientation * delta_q;
                current_orientation.normalize();

                // Update orientation in odometry
                odom_.pose.pose.orientation = tf2::toMsg(current_orientation);

                // Update timestamps in odometry and pose
                odom_.header.stamp = simulated_time_;

                // Make a copy of odom for thread safety
                nav_msgs::msg::Odometry odom_copy = odom_;

                // Capture current_time and new_message_ for the plugins
                rclcpp::Time current_time = simulated_time_;
                bool has_new_message = new_message_;

                // Reset new_message_ flag
                new_message_ = false;

                // Dispatch plugin tick calls asynchronously
                auto gps_future = std::async(std::launch::async, [this, current_time, odom_copy, has_new_message]() {
                    gps_plugin_.tick(current_time, odom_copy, has_new_message);
                });

                auto imu_future = std::async(std::launch::async, [this, current_time, odom_copy, has_new_message]() {
                    imu_plugin_.tick(current_time, odom_copy, has_new_message);
                });
                // Publish simulated clock
                rosgraph_msgs::msg::Clock clock_msg;
                clock_msg.clock = simulated_time_;
                clock_pub_->publish(clock_msg);

                // Update last_update_
                last_update_ = simulated_time_;

                // Reset twist to zero (optional, based on your simulation logic)
                // If you intend to continuously apply target velocities, you might want to remove this.
                auto zero_twist = geometry_msgs::msg::Twist();
                target_twist_ = zero_twist;
            }

    };
} // namespace sim

#endif // SIM_HPP
