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

#include "farmbot_flatlands/robot.hpp"
#include "farmbot_flatlands/environment.hpp"

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
#include <iostream>
#include <random>
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

            // Diagnostic Updater
            diagnostic_updater::Updater updater_;
            diagnostic_msgs::msg::DiagnosticStatus status;

            // Robot instance
            std::shared_ptr<Robot> robot_;

            // Environment instance
            std::shared_ptr<Environment> env_;

            // Seed for random number generator
            std::random_device rd_;

        public:
            SIM(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            ~SIM();
            void start();

        private:
            void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
            void update_loop();
            nav_msgs::msg::Odometry random_odom(int min, int max);
            // Service callbacks
            void pause_callback(const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response);
            void speed_callback(const std::shared_ptr<Value::Request> request, std::shared_ptr<Value::Response> response);
    };

    // Implementation of SIM class methods

    inline SIM::SIM(const rclcpp::NodeOptions & options) : Node("simulator", options), updater_(this){
        // Declare and get parameters
        this->declare_parameter<double>("publish_rate", 10.0);
        this->get_parameter("publish_rate", publish_rate_);

        // Diagnostic
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Not initialized";

        // Initialize simulated time
        simulated_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        last_update_ = simulated_time_;

        // Subscriber for velocity commands
        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&SIM::vel_callback, this, _1));

        // Play / Pause Service
        pause_service_ = this->create_service<Trigger>("/sim/pause", std::bind(&SIM::pause_callback, this, _1, _2));
        set_speed_service_ = this->create_service<Value>("/sim/speed", std::bind(&SIM::speed_callback, this, _1, _2));

        // Publisher for simulated clock
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // Initialize maximum accelerations (tunable parameters)
        this->declare_parameter<double>("simulation_speed", 1.0);
        this->get_parameter("simulation_speed", simulation_speed_);

        RCLCPP_INFO(this->get_logger(), "SIM initialized.");
    }

    inline SIM::~SIM(){
        if (loop_timer_) { loop_timer_->cancel(); }
        RCLCPP_INFO(this->get_logger(), "Simulator stopped.");
    }

    inline void SIM::start(){
        // Initialize Environment
        env_ = std::make_shared<Environment>(this->shared_from_this());
        // Initialize Robot with initial heading (converted to radians)
        robot_ = std::make_shared<Robot>(this->shared_from_this(), env_ , 0.0);
        // Start the simulator
        loop_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_), std::bind(&SIM::update_loop, this));
        // Initialize the robot
        robot_->init(random_odom(-100, 100));
    }

    inline void SIM::vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        if (paused_) { return; }
        robot_->set_twist(*msg);
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
        robot_->update(delta_t, simulated_time_);

        // Publish /clock message
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock = simulated_time_;
        clock_pub_->publish(clock_msg);

        // Update last_update_
        last_update_ = simulated_time_;
    }

    nav_msgs::msg::Odometry SIM::random_odom(int min=100, int max=200){
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
