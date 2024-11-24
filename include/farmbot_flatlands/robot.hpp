#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "muli/math.h"
#include "muli/rigidbody.h"
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
#include <random>

#include "farmbot_flatlands/world.hpp"
#include "farmbot_flatlands/utils/utils.hpp"
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
            std::shared_ptr<World> world_;
            muli::RigidBody* robot_;
            rclcpp::Logger logger_;

            std::string name_;

            // Plugins
            std::shared_ptr<plugins::GPSPlugin> gps_plugin_;
            std::shared_ptr<plugins::IMUPlugin> imu_plugin_;
            std::shared_ptr<plugins::GyroPlugin> gyro_plugin_;
            std::shared_ptr<plugins::CompassPlugin> compass_plugin_;
            // Odometry ad datum
            nav_msgs::msg::Odometry odom_;
            sensor_msgs::msg::NavSatFix datum_;
            // Velocity Control Variables
            geometry_msgs::msg::Twist current_twist_;
            geometry_msgs::msg::Twist target_twist_;
            double max_linear_vel_ = 1;     // meters per second
            double max_angular_vel_ = 1;    // radians per second
            double max_linear_accel_;   // meters per second squared
            double max_angular_accel_;  // radians per second squared
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

            void update_alt(double delta_t, const rclcpp::Time & current_time);
            void update_ori(double delta_t, const rclcpp::Time & current_time);
        public:
            Robot(std::string name, const rclcpp::Node::SharedPtr& node, std::shared_ptr<World> world);
            void init(nav_msgs::msg::Odometry odom = random_odom(-100, 100));
            void set_twist(const geometry_msgs::msg::Twist& twist);
            void update(double delta_t, const rclcpp::Time & current_time);
            // getters
            nav_msgs::msg::Odometry get_odom() const;
            sensor_msgs::msg::NavSatFix get_datum() const;
            // Getter for robot's position (x, y, z)
            std::tuple<double, double, double> get_position() const;
            static nav_msgs::msg::Odometry random_odom(int min, int max);
    };

    // Implementation of Robot class methods
    inline Robot::Robot(
        std::string name,
        const rclcpp::Node::SharedPtr& node,
        std::shared_ptr<World> world):
            node_(node),
            world_(world),
            logger_(node->get_logger()),
            name_(name)
        {
        twist_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
            name_ + "/cmd_vel", 10, std::bind(&Robot::set_twist, this, std::placeholders::_1));
        // Initialize odometry message
        odom_.header.frame_id = "world";
        odom_.child_frame_id = "base_link";

        datum_ = world_->settings.get_datum();
        // Initialize current and target velocities to zero
        current_twist_ = geometry_msgs::msg::Twist();
        target_twist_ = geometry_msgs::msg::Twist();

        // Get parameters for maximum accelerations
        node->get_parameter("max_linear_accel", max_linear_accel_);
        node->get_parameter("max_angular_accel", max_angular_accel_);
    }

    inline void Robot::init(nav_msgs::msg::Odometry odom) {
        odom_ = odom;
        // Create GPS Plugin
        gps_plugin_ = std::make_shared<plugins::GPSPlugin>(node_, name_ + "/gnss", get_datum());
        // Create IMU Plugin
        imu_plugin_ = std::make_shared<plugins::IMUPlugin>(node_, name_ + "/imu", get_odom());
        // Create Gyro Plugin
        gyro_plugin_ = std::make_shared<plugins::GyroPlugin>(node_, name_ + "/gyro", get_odom());
        // Create Compass Plugin
        compass_plugin_ = std::make_shared<plugins::CompassPlugin>(node_, name_ + "/compass", get_odom());

        // Add robot to world
        robot_ = world_->add_robot(1.0f, 0.5f, name_);
        robot_->SetPosition(odom_.pose.pose.position.x, odom_.pose.pose.position.y);
        // theta
        auto theta = angle::theta(odom_.pose.pose.orientation);
        robot_->SetRotation(theta);
    }

    inline void Robot::set_twist(const geometry_msgs::msg::Twist& twist) {
        target_twist_ = twist;
    }

    inline void Robot::update(double delta_t, const rclcpp::Time & current_time) {
        update_ori(delta_t, current_time);
        // update_alt(delta_t, current_time);
        // Dispatch plugin tick calls asynchronously
        auto odom_copy = get_odom();
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


    inline void Robot::update_alt(double delta_t, const rclcpp::Time & current_time) {
        // Treat target_twist_ as acceleration commands
        robot_->SetLinearVelocity(target_twist_.linear.x, target_twist_.linear.y);
        robot_->SetAngularVelocity(target_twist_.angular.z);

        // Get updated position and rotation from Muli
        auto position = robot_->GetPosition();
        auto rotation = robot_->GetRotation();

        // Update odometry message
        odom_.pose.pose.position.x = position.x;
        odom_.pose.pose.position.y = position.y;
        odom_.pose.pose.position.z = 0.0; // Assuming 2D simulation

        // Convert rotation to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, rotation.GetAngle()); // Assuming 2D rotation around Z axis
        odom_.pose.pose.orientation = tf2::toMsg(q);

        // Update twist in odometry
        odom_.twist.twist = current_twist_;
    }


    inline void Robot::update_ori(double delta_t, const rclcpp::Time & current_time) {
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
    }

    inline nav_msgs::msg::Odometry Robot::get_odom() const {
        return odom_;
    }

    inline sensor_msgs::msg::NavSatFix Robot::get_datum() const {
        return datum_;
    }

    inline std::tuple<double, double, double> Robot::get_position() const {
        return std::make_tuple(
            odom_.pose.pose.position.x,
            odom_.pose.pose.position.y,
            odom_.pose.pose.position.z
        );
    }

    inline nav_msgs::msg::Odometry Robot::random_odom(int min=100, int max=200){
        std::random_device rd_;
        std::mt19937 gen(rd_());
        std::uniform_int_distribution<int> dist(min, max);
        nav_msgs::msg::Odometry odom;
        odom.pose.pose.position.x = dist(gen)*1.0;
        odom.pose.pose.position.y = dist(gen)*1.0;
        odom.pose.pose.position.z = 0.0;
        // random orientation
        std::uniform_real_distribution<double> dist2(-M_PI, M_PI);
        odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), dist2(gen)));
        return odom;
    }
} // namespace sim

#endif // ROBOT_HPP
