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
// #include "farmbot_flatlands/envi.hpp"
#include "farmbot_flatlands/world.hpp"

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
using diag = diagnostic_msgs::msg::DiagnosticStatus;

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
            //muli world
            WorldSettings world_settings_;
            std::shared_ptr<World> world_;

            // Parameters
            double simulation_speed_;
            std::vector<double> datum_param_;
            bool paused_ = true;
            double publish_rate_;

            // Time tracking
            rclcpp::Time last_update_;
            rclcpp::Time simulated_time_;

            // ROS Interfaces
            rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
            // Pause/Resume Service
            rclcpp::Service<farmbot_interfaces::srv::Trigger>::SharedPtr pause_service_;
            rclcpp::Service<farmbot_interfaces::srv::Value>::SharedPtr set_speed_service_;
            // Timer
            rclcpp::TimerBase::SharedPtr loop_timer_;
            // Diagnostic Updater
            diagnostic_updater::Updater updater_;
            diagnostic_msgs::msg::DiagnosticStatus status;
            rclcpp::TimerBase::SharedPtr diagnostic_timer_;
            // Vector of robots
            int num_robots_;
            std::vector<std::shared_ptr<Robot>> robots_;
            // Environment instance
            // std::shared_ptr<Environment> env_;
            // Seed for random number generator
            std::random_device rd_;

        public:
            SIM(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            ~SIM();
            void start_simulation();
            void create_world();

        private:
            void update_loop();
            nav_msgs::msg::Odometry random_odom(int min, int max);
            // Service callbacks
            void pause_callback(const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response);
            void speed_callback(const std::shared_ptr<Value::Request> request, std::shared_ptr<Value::Response> response);
            // Diagnostics
            void diagnostic_callback();
            void check_system(diagnostic_updater::DiagnosticStatusWrapper &stat);
    };

    // Implementation of SIM class methods

    inline SIM::SIM(const rclcpp::NodeOptions & options) : Node("simulator", options), updater_(this){
        // Declare and get parameters
        this->declare_parameter<double>("publish_rate", 10.0);
        this->get_parameter("publish_rate", publish_rate_);
        this->declare_parameter<std::vector<double>>("datum", {0.0, 0.0, 0.0});
        this->get_parameter("datum", datum_param_);
        this->declare_parameter<int>("num_robots", 1);
        this->get_parameter("num_robots", num_robots_);
        this->declare_parameter<double>("max_angular_accel", 0.7);  // rad/s²
        this->declare_parameter<double>("max_linear_accel", 0.7);   // m/s²

        // Diagnostic
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Not initialized";

        // Initialize simulated time
        simulated_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        last_update_ = simulated_time_;

        // Play / Pause Service
        pause_service_ = this->create_service<Trigger>("/sim/pause", std::bind(&SIM::pause_callback, this, _1, _2));
        set_speed_service_ = this->create_service<Value>("/sim/speed", std::bind(&SIM::speed_callback, this, _1, _2));

        // Publisher for simulated clock
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // Initialize maximum accelerations (tunable parameters)
        this->declare_parameter<double>("simulation_speed", 1.0);
        this->get_parameter("simulation_speed", simulation_speed_);

        // Diagnostics
        updater_.setHardwareID(static_cast<std::string>(this->get_namespace()) + "/sim");
        updater_.add("Navigation Status", this, &SIM::check_system);
        diagnostic_timer_ = this->create_wall_timer(1s, std::bind(&SIM::diagnostic_callback, this));

        RCLCPP_INFO(this->get_logger(), "SIM initialized.");
    }

    inline SIM::~SIM(){
        if (loop_timer_) { loop_timer_->cancel(); }
        RCLCPP_INFO(this->get_logger(), "Simulator stopped.");
    }

    inline void SIM::create_world(){
        world_settings_.apply_gravity = false;
        world_settings_.set_datum(datum_param_);
        world_ = std::make_shared<World>(world_settings_, this->shared_from_this());
    }

    inline void SIM::start_simulation(){
        // Initialize Environment
        // env_ = std::make_shared<Environment>(this->shared_from_this(), world_);
        // Initialize Robots
        for (int i = 0; i < num_robots_; i++){
            std::string robot_name = "robot" + std::to_string(i);
            robots_.push_back(std::make_shared<Robot>(robot_name, this->shared_from_this(), world_));
            robots_[i]->init();
        }
        // Start the simulator
        loop_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_), std::bind(&SIM::update_loop, this));
        paused_ = false;
    }

    inline void SIM::diagnostic_callback() {
        updater_.force_update();
    }

    inline void SIM::check_system(diagnostic_updater::DiagnosticStatusWrapper &stat) {
        stat.summary(status.level, status.message);
    }

    inline void SIM::update_loop(){
        if (paused_) { return; }
        status.level = paused_ ? diag::WARN : diag::OK;
        status.message = paused_ ? "Paused" : "Running";

        // Increment simulated time
        double time_increment = (1.0 / publish_rate_) * simulation_speed_;
        simulated_time_ += rclcpp::Duration::from_seconds(time_increment);

        // Compute time difference
        double delta_t = time_increment;
        world_->Step(time_increment);

        // Update robot state
        for (int i = 0; i < num_robots_; i++){
            robots_[i]->update(delta_t, simulated_time_);
        }

        // Publish /clock message
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
