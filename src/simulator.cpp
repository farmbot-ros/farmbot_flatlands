#include "rclcpp/rclcpp.hpp"

// Message Types
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

// Plugins (Header-Only)
#include "farmbot_flatlands/gps_plugin.hpp"
#include "farmbot_flatlands/imu_plugin.hpp"

// TF2 Headers
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Standard Libraries
#include <curl/curl.h>
#include <ios>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <cmath>
#include <tuple>

using namespace std::chrono_literals;
namespace sim {
    class SIM : public rclcpp::Node {
        private:
            // Robot state
            nav_msgs::msg::Odometry odom_;
            std_msgs::msg::Float32 heading_;
            std_msgs::msg::Float32 theta_;
            sensor_msgs::msg::NavSatFix fix_;

            // Quaterion
            tf2::Quaternion quat;

            // Parameters
            double publish_rate_;
            std::string velocity_topic_;
            std::string fix_topic_;
            std::string imu_topic_;
            std::string heading_topic_;

            double latitude_;
            double longitude_;
            double altitude_;
            double heading_deg_;

            // Time tracking
            rclcpp::Time last_update_;
            rclcpp::Time simulated_time_;

            // ROS Interfaces
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
            rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_pub;

            // Timer
            rclcpp::TimerBase::SharedPtr loop_timer_;

            // GPS Plugin
            plugins::GPSPlugin gps_plugin_;
            // IMU Plugin
            plugins::IMUPlugin imu_plugin_;


        public:
            SIM(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("simulator", options) {
                // Declare and get parameters
                // Declare and get parameters
                this->declare_parameter<double>("publish_rate", 10.0);
                this->get_parameter("publish_rate", publish_rate_);
                this->declare_parameter<std::string>("velocity_topic", "cmd_vel");
                this->get_parameter("velocity_topic", velocity_topic_);
                this->declare_parameter<std::string>("heading_topic", "heading");
                this->get_parameter("heading_topic", heading_topic_);
                this->declare_parameter<std::string>("imu_topic", "imu/data");
                this->get_parameter("imu_topic", imu_topic_);
                this->declare_parameter<std::string>("position_topic", "fix");
                this->get_parameter("position_topic", fix_topic_);
                this->declare_parameter<double>("latitude", 0.0);
                this->get_parameter("latitude", latitude_);
                this->declare_parameter<double>("longitude", 0.0);
                this->get_parameter("longitude", longitude_);
                this->declare_parameter<double>("altitude", 0.0);
                this->get_parameter("altitude", altitude_);
                this->declare_parameter<double>("heading", 0.0);
                this->get_parameter("heading", heading_deg_);

                // Convert initial heading from degrees to radians
                theta_.data = heading_deg_ * M_PI / 180.0;

                // Initialize odometry message
                odom_.header.frame_id = "world";
                odom_.pose.pose.position.x = 0.0;
                odom_.pose.pose.position.y = 0.0;
                odom_.pose.pose.position.z = 0.0;

                quat.setRPY(0, 0, theta_.data);
                odom_.pose.pose.orientation = tf2::toMsg(quat);

                odom_.twist.twist.linear.x = 0.0;
                odom_.twist.twist.linear.y = 0.0;
                odom_.twist.twist.linear.z = 0.0;
                odom_.twist.twist.angular.x = 0.0;
                odom_.twist.twist.angular.y = 0.0;
                odom_.twist.twist.angular.z = 0.0;

                // Initialize heading
                heading_.data = heading_deg_;


                // Initialize pose
                fix_.header.frame_id = "world";
                fix_.latitude = latitude_;
                fix_.longitude = longitude_;
                fix_.altitude = altitude_;

                RCLCPP_INFO(this->get_logger(), "GPS: Latitude: %f, Longitude: %f, Altitude: %f", fix_.latitude, fix_.longitude, fix_.altitude);
                // Subscriber for velocity commands
                vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(velocity_topic_, 10, std::bind(&SIM::vel_callback, this, std::placeholders::_1));
                // heading publisher
                heading_pub = this->create_publisher<std_msgs::msg::Float32>(heading_topic_, 10);
                // Publisher for simulated clock
                clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
                // Initialize simulated time
                simulated_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
                last_update_ = simulated_time_;
                RCLCPP_INFO(this->get_logger(), "SIM initialized.");
            }

            ~SIM(){
                loop_timer_->cancel();
                RCLCPP_INFO(this->get_logger(), "Simulator stopped.");
            }

            void start(){
                // Create GPS Plugin
                gps_plugin_ = plugins::GPSPlugin(this->shared_from_this(), fix_topic_, fix_);
                // Create IMU Plugin
                imu_plugin_ = plugins::IMUPlugin(this->shared_from_this(), imu_topic_, odom_);
                // Start the simulator
                loop_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_), std::bind(&SIM::update_loop, this));
            }

        private:
            void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
                odom_.twist.twist = *msg;
            }

            void update_loop(){
                // Increment simulated time
                simulated_time_ += rclcpp::Duration::from_seconds(1.0 / publish_rate_);

                // Compute time difference
                rclcpp::Duration dt = simulated_time_ - last_update_;
                double delta_t = dt.seconds();
                if (delta_t <= 0.0) { delta_t = 1.0 / publish_rate_; }

                // Update robot's position and orientation based on current velocities
                double delta_x = (odom_.twist.twist.linear.x * std::cos(theta_.data) - odom_.twist.twist.linear.y * std::sin(theta_.data)) * delta_t;
                double delta_y = (odom_.twist.twist.linear.x * std::sin(theta_.data) + odom_.twist.twist.linear.y * std::cos(theta_.data)) * delta_t;
                double delta_z = odom_.twist.twist.linear.z * delta_t;
                double delta_th = odom_.twist.twist.angular.z * delta_t;

                odom_.pose.pose.position.x += delta_x;
                odom_.pose.pose.position.y += delta_y;
                odom_.pose.pose.position.z += delta_z;

                theta_.data += delta_th;
                theta_.data = std::atan2(std::sin(theta_.data), std::cos(theta_.data)); // Normalize angle

                // Update orientation quaternion
                quat.setRPY(0, 0, theta_.data);
                odom_.pose.pose.orientation = tf2::toMsg(quat);

                // Update heading
                heading_.data = theta_.data * 180.0 / M_PI;
                heading_pub->publish(heading_);

                // Update timestamps in odometry and pose
                odom_.header.stamp = simulated_time_;

                // Call plugins' tick functions
                gps_plugin_.tick(simulated_time_, odom_);
                imu_plugin_.tick(simulated_time_, odom_);

                // Publish simulated clock
                rosgraph_msgs::msg::Clock clock_msg;
                clock_msg.clock = simulated_time_;
                clock_pub_->publish(clock_msg);

                // Update last_update_
                last_update_ = simulated_time_;
            }
    };
} // namespace mobile_robot_simulator

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {{"use_sim_time", true}}
    );

    auto simulator = std::make_shared<sim::SIM>(options);
    simulator->start();

    rclcpp::spin(simulator);
    rclcpp::shutdown();
    return 0;
}
