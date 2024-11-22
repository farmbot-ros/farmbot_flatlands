#ifndef SIM_HPP
#define SIM_HPP

#include "rclcpp/rclcpp.hpp"

// Message Types
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

// Service Types
#include "farmbot_interfaces/srv/trigger.hpp"
#include "farmbot_interfaces/srv/value.hpp"

// Plugins (Header-Only)
#include "farmbot_flatlands/plugins/gps.hpp"
#include "farmbot_flatlands/plugins/imu.hpp"
#include "farmbot_flatlands/plugins/gyro.hpp"
#include "farmbot_flatlands/plugins/compass.hpp"

#include "farmbot_flatlands/robot.hpp"

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
using namespace std::placeholders;

namespace sim {
    using Trigger = farmbot_interfaces::srv::Trigger;
    using Value = farmbot_interfaces::srv::Value;

    // SIM class manages simulation time and uses Robot class for robot state
    class SIM : public rclcpp::Node {
        private:
            double simulation_speed_;
            bool paused_;

            // Parameters
            double publish_rate_;
            std::string vel_topic_;
            std::string fix_topic_;
            std::string imu_topic_;
            std::string gyro_topic_;
            std::string compass_topic_;

            double latitude_;
            double longitude_;
            double altitude_;
            double heading_;

            // Fix message for GPS
            sensor_msgs::msg::NavSatFix fix_;

            // Time tracking
            rclcpp::Time last_update_;
            rclcpp::Time simulated_time_;

            // ROS Interfaces
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
            rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

            // Pause/Resume Service
            rclcpp::Service<farmbot_interfaces::srv::Trigger>::SharedPtr pause_service_;
            rclcpp::Service<farmbot_interfaces::srv::Value>::SharedPtr set_speed_service_;

            // Timer
            rclcpp::TimerBase::SharedPtr loop_timer_;
            bool new_message_ = false;

            // Diagnostic Updater
            diagnostic_updater::Updater updater_;
            diagnostic_msgs::msg::DiagnosticStatus status;

            // Plugins
            plugins::GPSPlugin gps_plugin_;
            plugins::IMUPlugin imu_plugin_;
            plugins::GyroPlugin gyro_plugin_;
            plugins::CompassPlugin compass_plugin_;

            // Robot instance
            std::shared_ptr<Robot> robot_;

        public:
            SIM(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            ~SIM();

            void start();

        private:
            void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
            void update_loop();

            // Service callbacks

            void pause_callback(const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response);
            void speed_callback(const std::shared_ptr<Value::Request> request, std::shared_ptr<Value::Response> response);

    };

    // Implementation of SIM class methods

    inline SIM::SIM(const rclcpp::NodeOptions & options) : Node("simulator", options), updater_(this){
        // Declare and get parameters
        this->declare_parameter<double>("publish_rate", 10.0);
        this->get_parameter("publish_rate", publish_rate_);
        this->declare_parameter<std::string>("velocity_topic", "cmd_vel");
        this->get_parameter("velocity_topic", vel_topic_);
        this->declare_parameter<std::string>("imu_topic", "imu");
        this->get_parameter("imu_topic", imu_topic_);
        this->declare_parameter<std::string>("gnss_topic", "gnss");
        this->get_parameter("gnss_topic", fix_topic_);
        this->declare_parameter<std::string>("gyro_topic", "gyro");
        this->get_parameter("gyro_topic", gyro_topic_);
        this->declare_parameter<std::string>("compass_topic", "compass");
        this->get_parameter("compass_topic", compass_topic_);
        this->declare_parameter<double>("latitude", 0.0);
        this->get_parameter("latitude", latitude_);
        this->declare_parameter<double>("longitude", 0.0);
        this->get_parameter("longitude", longitude_);
        this->declare_parameter<double>("altitude", 0.0);
        this->get_parameter("altitude", altitude_);
        this->declare_parameter<double>("heading", 0.0);
        this->get_parameter("heading", heading_);

        // Diagnostic
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Not initialized";

        // Initialize simulated time
        simulated_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        last_update_ = simulated_time_;

        // Subscriber for velocity commands
        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>( vel_topic_, 10, std::bind(&SIM::vel_callback, this, _1));

        // Play / Pause Service
        pause_service_ = this->create_service<Trigger>("/sim/pause", std::bind(&SIM::pause_callback, this, _1, _2));
        set_speed_service_ = this->create_service<Value>("/sim/speed", std::bind(&SIM::speed_callback, this, _1, _2));

        // Publisher for simulated clock
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // Initialize maximum accelerations (tunable parameters)
        this->declare_parameter<double>("simulation_speed", 1.0);
        this->get_parameter("simulation_speed", simulation_speed_);

        // Initialize fix message
        fix_.header.frame_id = "gps";
        fix_.latitude = latitude_;
        fix_.longitude = longitude_;
        fix_.altitude = altitude_;
        fix_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

        RCLCPP_INFO(this->get_logger(), "SIM initialized.");
    }

    inline SIM::~SIM(){
        if (loop_timer_) { loop_timer_->cancel(); }
        RCLCPP_INFO(this->get_logger(), "Simulator stopped.");
    }

    inline void SIM::start(){
        // Initialize Robot with initial heading (converted to radians)
        robot_ = std::make_shared<Robot>(this->shared_from_this(), heading_ * M_PI / 180.0);
        // Create GPS Plugin
        gps_plugin_ = plugins::GPSPlugin(this->shared_from_this(), fix_topic_, fix_);
        // Create IMU Plugin
        imu_plugin_ = plugins::IMUPlugin(this->shared_from_this(), imu_topic_, robot_->get_odom());
        // Create Gyro Plugin
        gyro_plugin_ = plugins::GyroPlugin(this->shared_from_this(), gyro_topic_, robot_->get_odom());
        // Create Compass Plugin
        compass_plugin_ = plugins::CompassPlugin(this->shared_from_this(), compass_topic_, robot_->get_odom());
        // Start the simulator
        loop_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate_),
            std::bind(&SIM::update_loop, this)
        );
    }

    inline void SIM::vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        if (paused_) { return; }
        robot_->set_twist(*msg);
        new_message_ = true;
    }

    inline void SIM::update_loop(){
        if (paused_) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = "Paused";
            return;
        }
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "Running";

        // Increment simulated time
        double time_increment = (1.0 / publish_rate_) * simulation_speed_;
        simulated_time_ += rclcpp::Duration::from_seconds(time_increment);

        // Compute time difference
        double delta_t = time_increment;

        // Update robot state
        robot_->update(delta_t);

        // Get odometry from robot
        nav_msgs::msg::Odometry odom_copy = robot_->get_odom();

        // Update timestamps in odometry
        odom_copy.header.stamp = simulated_time_;
        odom_copy.child_frame_id = "base_link";

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
        auto gyro_future = std::async(std::launch::async, [this, current_time, odom_copy, has_new_message]() {
            gyro_plugin_.tick(current_time, odom_copy, has_new_message);
        });
        auto compass_future = std::async(std::launch::async, [this, current_time, odom_copy, has_new_message]() {
            compass_plugin_.tick(current_time, odom_copy, has_new_message);
        });

        // Publish simulated clock
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock = simulated_time_;
        clock_pub_->publish(clock_msg);

        // Update last_update_
        last_update_ = simulated_time_;
    }


    inline void SIM::pause_callback(const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response){
        paused_ = !paused_;
    }

    inline void SIM::speed_callback(const std::shared_ptr<Value::Request> request, std::shared_ptr<Value::Response> response){
        double value = request->value.data;
        if (value > 0.0 && value < 10.0){
            simulation_speed_ = value;
        }
    }
} // namespace sim

#endif // SIM_HPP
