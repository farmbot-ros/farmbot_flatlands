#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "rclcpp/rclcpp.hpp"

// Message Types
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// TF2 Headers
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Standard Libraries
#include <memory>
#include <sensor_msgs/msg/detail/nav_sat_fix__struct.hpp>
#include <string>
#include <cmath>
#include <tuple>
#include <algorithm> // For std::clamp

#include "farmbot_flatlands/environment.hpp"
#include "farmbot_flatlands/plugins/plugin.hpp"

#include "farmbot_flatlands/plugins/gps.hpp"
#include "farmbot_flatlands/plugins/imu.hpp"
#include "farmbot_flatlands/plugins/gyro.hpp"
#include "farmbot_flatlands/plugins/compass.hpp"

using namespace std::chrono_literals;

namespace sim {
    // Robot class handles odometry and robot state updates
    class Robot {
        private:
            //Pointer to the node
            rclcpp::Node::SharedPtr node_;
            std::shared_ptr<Environment> environment_;
            rclcpp::Logger logger_;

            std::string name_;

            // Plugins
            std::shared_ptr<plugins::GPSPlugin> gps_plugin_;
            std::shared_ptr<plugins::IMUPlugin> imu_plugin_;
            std::shared_ptr<plugins::GyroPlugin> gyro_plugin_;
            std::shared_ptr<plugins::CompassPlugin> compass_plugin_;
            // Odometry data
            nav_msgs::msg::Odometry odom_;
            // Velocity Control Variables
            geometry_msgs::msg::Twist current_twist_;
            geometry_msgs::msg::Twist target_twist_;
            double max_linear_accel_;   // meters per second squared
            double max_angular_accel_;  // radians per second squared

        public:
            Robot(const rclcpp::Node::SharedPtr& node, std::shared_ptr<Environment> environment);
            void init(std::string name, nav_msgs::msg::Odometry odom);
            void set_twist(const geometry_msgs::msg::Twist& twist);
            void update(double delta_t, const rclcpp::Time & current_time);
            // Getter for odometry
            nav_msgs::msg::Odometry get_odom() const;
            // Getter for robot's position (x, y, z)
            std::tuple<double, double, double> get_position() const;
    };

    // Implementation of Robot class methods
    inline Robot::Robot(
        const rclcpp::Node::SharedPtr& node,
        std::shared_ptr<Environment> environment)
        : node_(node), environment_(environment), logger_(node->get_logger()) {
        // Initialize odometry message
        odom_.header.frame_id = "world";
        odom_.child_frame_id = "base_link";

        // Initialize current and target velocities to zero
        current_twist_ = geometry_msgs::msg::Twist();
        target_twist_ = geometry_msgs::msg::Twist();

        // Get parameters for maximum accelerations
        node->get_parameter("max_linear_accel", max_linear_accel_);
        node->get_parameter("max_angular_accel", max_angular_accel_);
    }

    inline void Robot::init(std::string name, nav_msgs::msg::Odometry odom) {
        name_ = name;
        // Set initial odometry
        odom_ = odom;
        // Create GPS Plugin
        gps_plugin_ = std::make_shared<plugins::GPSPlugin>(node_, name_ + "/gnss", environment_->get_datum());
        // Create IMU Plugin
        imu_plugin_ = std::make_shared<plugins::IMUPlugin>(node_, name_ + "/imu", get_odom());
        // Create Gyro Plugin
        gyro_plugin_ = std::make_shared<plugins::GyroPlugin>(node_, name_ + "/gyro", get_odom());
        // Create Compass Plugin
        compass_plugin_ = std::make_shared<plugins::CompassPlugin>(node_, name_ + "/compass", get_odom());
    }

    inline void Robot::set_twist(const geometry_msgs::msg::Twist& twist) {
        target_twist_ = twist;
    }

    inline void Robot::update(double delta_t, const rclcpp::Time & current_time) {
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

        // Check for collision before updating position
        geometry_msgs::msg::Point proposed_position;
        proposed_position.x = odom_.pose.pose.position.x + delta_pos.x();
        proposed_position.y = odom_.pose.pose.position.y + delta_pos.y();
        proposed_position.z = odom_.pose.pose.position.z + delta_pos.z();

        if (environment_->is_collision(proposed_position)) {
            RCLCPP_WARN(logger_, "Collision detected at position (%f, %f)", proposed_position.x, proposed_position.y);
            // Handle collision: stop the robot
            current_twist_.linear.x = 0.0;
            current_twist_.linear.y = 0.0;
            current_twist_.linear.z = 0.0;
            current_twist_.angular.x = 0.0;
            current_twist_.angular.y = 0.0;
            current_twist_.angular.z = 0.0;
            target_twist_ = current_twist_; // Update target to current to prevent further movement
        } else {
            // No collision, update position
            odom_.pose.pose.position.x = proposed_position.x;
            odom_.pose.pose.position.y = proposed_position.y;
            odom_.pose.pose.position.z = proposed_position.z;

            // Update orientation
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

            auto odom_copy = get_odom();

            // Dispatch plugin tick calls asynchronously
            auto gps_future = std::async(std::launch::async, [this, current_time, odom_copy]() {
                gps_plugin_->tick(current_time, odom_copy);
            });
            auto imu_future = std::async(std::launch::async, [this, current_time, odom_copy]() {
                imu_plugin_->tick(current_time, odom_copy);
            });
            auto gyro_future = std::async(std::launch::async, [this, current_time, odom_copy]() {
                gyro_plugin_->tick(current_time, odom_copy);
            });
            auto compass_future = std::async(std::launch::async, [this, current_time, odom_copy]() {
                compass_plugin_->tick(current_time, odom_copy);
            });
        }
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
} // namespace sim

#endif // ROBOT_HPP
