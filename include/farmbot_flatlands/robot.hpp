
#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "rclcpp/rclcpp.hpp"

// Message Types
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

// Service Types
#include "farmbot_interfaces/srv/trigger.hpp"
#include "farmbot_interfaces/srv/value.hpp"

// TF2 Headers
#include "tf2/LinearMath/Quaternion.h"
#include <farmbot_interfaces/srv/detail/trigger__struct.hpp>
#include <farmbot_interfaces/srv/detail/value__struct.hpp>
#include <rclcpp/timer.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// ROS Diagnostics
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

// Standard Libraries
#include <memory>
#include <string>
#include <cmath>
#include <tuple>
#include <future> // For std::async
#include <algorithm> // For std::clamp

using namespace std::chrono_literals;

namespace sim {
    // Robot class handles odometry and robot state updates
    class Robot {
        private:
            // Odometry data
            nav_msgs::msg::Odometry odom_;
            // Velocity Control Variables
            geometry_msgs::msg::Twist current_twist_;
            geometry_msgs::msg::Twist target_twist_;
            double max_linear_accel_;   // meters per second squared
            double max_angular_accel_;  // radians per second squared
        public:
            Robot(const rclcpp::Node::SharedPtr& node, double initial_heading);
            void set_twist(const geometry_msgs::msg::Twist& twist);
            void update(double delta_t);
            // Getter for odometry
            nav_msgs::msg::Odometry get_odom() const;
            // Getter for robot's position (x, y, z)
            std::tuple<double, double, double> get_position() const;
    };

    // Implementation of Robot class methods
    inline Robot::Robot(const rclcpp::Node::SharedPtr& node, double initial_heading=0.0){
        // Initialize odometry message
        odom_.header.frame_id = "world";
        odom_.child_frame_id = "base_link";

        // Set initial orientation based on heading
        tf2::Quaternion quat;
        quat.setRPY(0, 0, initial_heading);
        odom_.pose.pose.orientation = tf2::toMsg(quat);

        // Initialize current and target velocities to zero
        current_twist_ = geometry_msgs::msg::Twist();
        target_twist_ = geometry_msgs::msg::Twist();

        // Get parameters for maximum accelerations
        node->declare_parameter<double>("max_linear_accel", 0.7);   // m/s²
        node->get_parameter("max_linear_accel", max_linear_accel_);
        node->declare_parameter<double>("max_angular_accel", 0.7);  // rad/s²
        node->get_parameter("max_angular_accel", max_angular_accel_);
    }

    inline void Robot::set_twist(const geometry_msgs::msg::Twist& twist) {
        target_twist_ = twist;
    }

    inline void Robot::update(double delta_t){
        // Calculate required acceleration
        auto calc_accel = [](double target, double current, double max_accel, double dt) {
            double accel = (target - current) / dt;
            return std::clamp(accel, -max_accel, max_accel);
        };

        // Update linear velocities
        double accel_x = calc_accel(target_twist_.linear.x, current_twist_.linear.x, max_linear_accel_, delta_t);
        double accel_y = calc_accel(target_twist_.linear.y, current_twist_.linear.y, max_linear_accel_, delta_t);
        double accel_z = calc_accel(target_twist_.linear.z, current_twist_.linear.z, max_linear_accel_, delta_t);

        current_twist_.linear.x += accel_x * delta_t;
        current_twist_.linear.y += accel_y * delta_t;
        current_twist_.linear.z += accel_z * delta_t;

        // Update angular velocities
        double angular_accel_x = calc_accel(target_twist_.angular.x, current_twist_.angular.x, max_angular_accel_, delta_t);
        double angular_accel_y = calc_accel(target_twist_.angular.y, current_twist_.angular.y, max_angular_accel_, delta_t);
        double angular_accel_z = calc_accel(target_twist_.angular.z, current_twist_.angular.z, max_angular_accel_, delta_t);

        current_twist_.angular.x += angular_accel_x * delta_t;
        current_twist_.angular.y += angular_accel_y * delta_t;
        current_twist_.angular.z += angular_accel_z * delta_t;

        // Assign velocities to odometry
        odom_.twist.twist = current_twist_;

        // Update position and orientation
        tf2::Quaternion current_orientation;
        tf2::fromMsg(odom_.pose.pose.orientation, current_orientation);

        tf2::Vector3 linear_vel(current_twist_.linear.x, current_twist_.linear.y, current_twist_.linear.z);
        tf2::Vector3 linear_vel_world = tf2::quatRotate(current_orientation, linear_vel);

        tf2::Vector3 delta_pos = linear_vel_world * delta_t;

        odom_.pose.pose.position.x += delta_pos.x();
        odom_.pose.pose.position.y += delta_pos.y();
        odom_.pose.pose.position.z += delta_pos.z();

        tf2::Vector3 angular_vel(current_twist_.angular.x, current_twist_.angular.y, current_twist_.angular.z);
        double angular_speed = angular_vel.length();

        tf2::Quaternion delta_q;
        if (angular_speed > 1e-6) {
            tf2::Vector3 rotation_axis = angular_vel.normalized();
            double rotation_angle = angular_speed * delta_t;
            delta_q.setRotation(rotation_axis, rotation_angle);
        } else {
            delta_q = tf2::Quaternion(0, 0, 0, 1);
        }

        current_orientation *= delta_q;
        current_orientation.normalize();

        odom_.pose.pose.orientation = tf2::toMsg(current_orientation);
    }

    inline nav_msgs::msg::Odometry Robot::get_odom() const {
        return odom_;
    }

    inline std::tuple<double, double, double> Robot::get_position() const {
        return std::make_tuple(
            odom_.pose.pose.position.x,
            odom_.pose.pose.position.y,
            odom_.pose.pose.position.z
        );
    }
} // namespace robot

#endif // ROBOT_HPP_
