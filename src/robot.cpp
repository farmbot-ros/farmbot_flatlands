#include "rclcpp/rclcpp.hpp"  // Include the main ROS2 C++ client library

// Message Types
#include "geometry_msgs/msg/twist.hpp"             // Include Twist message for velocity commands
#include "nav_msgs/msg/odometry.hpp"               // Include Odometry message for robot state
#include "sensor_msgs/msg/nav_sat_fix.hpp"         // Include NavSatFix message for GPS data
#include "sensor_msgs/msg/imu.hpp"                 // Include IMU message for IMU data
#include "std_msgs/msg/float32.hpp"                // Include Float32 message for simple float data
#include "visualization_msgs/msg/marker.hpp"       // Include Marker message for visualization in RViz
#include "rosgraph_msgs/msg/clock.hpp"             // Include Clock message for simulated time

// TF2 Headers
#include "tf2/LinearMath/Quaternion.h"             // Include Quaternion for orientation calculations
#include "tf2_ros/transform_broadcaster.h"         // Include TransformBroadcaster for TF2
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // Include TF2 conversions for geometry messages

// Standard Libraries
#include <functional>                      // Include for std::function
#include <rclcpp/logging.hpp>              // Include for ROS2 logging
#include <rclcpp/visibility_control.hpp>
#include <string>                          // Include for std::string
#include <cmath>                           // Include for math functions
#include <tf2/LinearMath/Matrix3x3.h>      // Include Matrix3x3 for rotation matrices
#include <tf2/LinearMath/Vector3.h>        // Include Vector3 for 3D vectors
#include <tuple>                           // Include for std::tuple

namespace loc {  // Define a namespace 'loc' to encapsulate localization-related functions and constants

    // Constants
    const double R = 6378137.0;                // Earth's radius in meters (equatorial radius)
    const double f = 1.0 / 298.257223563;      // Flattening factor of the Earth
    const double e2 = 2 * f - f * f;           // Square of the Earth's eccentricity

    // Function to convert ENU coordinates to ECEF coordinates relative to a datum
    std::tuple<double, double, double> enu_to_ecef(std::tuple<double, double, double> enu, std::tuple<double, double, double> datum) {
        // Extract ENU and datum coordinates
        double xEast, yNorth, zUp;
        std::tie(xEast, yNorth, zUp) = enu;
        double latRef, longRef, altRef;
        std::tie(latRef, longRef, altRef) = datum;

        // Convert reference latitude and longitude from degrees to radians
        double latRef_rad = latRef * M_PI / 180.0;
        double longRef_rad = longRef * M_PI / 180.0;
        double cosLatRef = std::cos(latRef_rad);
        double sinLatRef = std::sin(latRef_rad);
        double cosLongRef = std::cos(longRef_rad);
        double sinLongRef = std::sin(longRef_rad);

        // Calculate the prime vertical radius of curvature at the reference point
        double NRef = R / std::sqrt(1.0 - e2 * sinLatRef * sinLatRef);

        // Calculate reference ECEF coordinates based on the datum
        double x0 = (NRef + altRef) * cosLatRef * cosLongRef;
        double y0 = (NRef + altRef) * cosLatRef * sinLongRef;
        double z0 = (NRef * (1 - e2) + altRef) * sinLatRef;

        // Reverse the ENU to ECEF transformation using rotation matrices
        double dx = -sinLongRef * xEast - sinLatRef * cosLongRef * yNorth + cosLatRef * cosLongRef * zUp;
        double dy = cosLongRef * xEast - sinLatRef * sinLongRef * yNorth + cosLatRef * sinLongRef * zUp;
        double dz = cosLatRef * yNorth + sinLatRef * zUp;

        // Calculate final ECEF coordinates by adding the differences to the reference ECEF
        double x = x0 + dx;
        double y = y0 + dy;
        double z = z0 + dz;

        // Return the ECEF coordinates as a tuple
        return std::make_tuple(x, y, z);
    }

    // Function to convert ECEF coordinates to GPS coordinates (latitude, longitude, altitude)
    std::tuple<double, double, double> ecef_to_gps(double x, double y, double z) {
        const double eps = 1e-12; // Convergence threshold for iterative calculation
        double longitude = std::atan2(y, x) * 180.0 / M_PI; // Calculate longitude in degrees

        // Initial approximation of latitude and height
        double p = std::sqrt(x * x + y * y); // Distance from the Z-axis
        double theta = std::atan2(z * R, p * (1 - f) * R); // Auxiliary value for latitude calculation
        double sinTheta = std::sin(theta);
        double cosTheta = std::cos(theta);

        // Initial estimate of latitude in radians
        double latitude = std::atan2(z + e2 * (1 - f) * R * sinTheta * sinTheta * sinTheta,
                                     p - e2 * R * cosTheta * cosTheta * cosTheta);
        // Calculate the prime vertical radius of curvature
        double N = R / std::sqrt(1.0 - e2 * std::sin(latitude) * std::sin(latitude));
        double altitude = p / std::cos(latitude) - N; // Initial estimate of altitude
        double latOld;

        // Iterate to refine the latitude and altitude estimates
        do {
            latOld = latitude;
            N = R / std::sqrt(1.0 - e2 * std::sin(latitude) * std::sin(latitude));
            altitude = p / std::cos(latitude) - N;
            latitude = std::atan2(z + e2 * N * std::sin(latitude), p); // Update latitude
        } while (std::abs(latitude - latOld) > eps); // Continue until convergence

        // Return the GPS coordinates as a tuple (latitude and longitude in degrees, altitude in meters)
        return std::make_tuple(latitude * 180.0 / M_PI, longitude, altitude);
    }

    // Function to convert ENU coordinates to GPS coordinates relative to a datum
    std::tuple<double, double, double> enu_to_gps(double xEast, double yNorth, double zUp, double latRef, double longRef, double altRef) {
        // First, convert ENU to ECEF coordinates using the datum
        std::tuple<double, double, double> ecef = enu_to_ecef(std::make_tuple(xEast, yNorth, zUp), std::make_tuple(latRef, longRef, altRef));
        // Then, convert ECEF to GPS coordinates
        return ecef_to_gps(std::get<0>(ecef), std::get<1>(ecef), std::get<2>(ecef));
    }
} // End of namespace loc

// Class definition for the MobileRobotSimulator node
class MobileRobotSimulator : public rclcpp::Node {
private:
    // Parameters
    double publish_rate;
    std::string velocity_topic_;
    std::string imu_topic_;
    double latitude_;
    double longitude_;
    double altitude_;
    double heading_;
    std::string position_topic_;
    std::string heading_topic_;

    // ROS2 Messages
    geometry_msgs::msg::Twist vel;
    nav_msgs::msg::Odometry odom;
    sensor_msgs::msg::NavSatFix pose;
    std_msgs::msg::Float32 heading;
    sensor_msgs::msg::Imu imu_msg_;

    // Time Variables
    rclcpp::Time last_vel;
    rclcpp::Time last_update;
    rclcpp::Time measure_time;
    rclcpp::Time simulated_time_;

    // State Variables
    bool is_running;
    double th;

    // ROS2 Interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_pub;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr loop_timer;

    geometry_msgs::msg::Twist prev_twist_;

public:
    // Constructor
    MobileRobotSimulator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("mobile_simulator", options), is_running(false), th(0.0) {

        // Declare and get parameters
        this->declare_parameter<double>("publish_rate", 10.0);
        this->declare_parameter<std::string>("velocity_topic", "cmd_vel");
        this->declare_parameter<std::string>("heading_topic", "heading");
        this->declare_parameter<std::string>("imu_topic", "imu/data");
        this->declare_parameter<std::string>("position_topic", "fix");
        this->declare_parameter<double>("latitude", 0.0);
        this->declare_parameter<double>("longitude", 0.0);
        this->declare_parameter<double>("altitude", 0.0);
        this->declare_parameter<double>("heading", 0.0);

        // Retrieve parameter values
        this->get_parameter("publish_rate", publish_rate);
        this->get_parameter("velocity_topic", velocity_topic_);
        this->get_parameter("heading_topic", heading_topic_);
        this->get_parameter("imu_topic", imu_topic_);
        this->get_parameter("position_topic", position_topic_);
        this->get_parameter("latitude", latitude_);
        this->get_parameter("longitude", longitude_);
        this->get_parameter("altitude", altitude_);
        this->get_parameter("heading", heading_);

        // Convert initial heading from degrees to radians
        th = heading_ * M_PI / 180.0;

        // Initialize odometry message
        odom.header.frame_id = "world";
        odom.pose.pose.position.x = 0.0;
        odom.pose.pose.position.y = 0.0;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, th);
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = 0.0;

        // Initialize velocity message
        vel = odom.twist.twist;

        // Initialize the pose message
        pose.header.frame_id = "world";
        pose.latitude = latitude_;
        pose.longitude = longitude_;
        pose.altitude = altitude_;

        // Initialize the heading message
        heading.data = heading_;

        // Create subscriber for velocity commands
        vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            velocity_topic_, 5,
            std::bind(&MobileRobotSimulator::vel_callback, this, std::placeholders::_1));

        // Create publishers
        gps_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>(position_topic_, 10);
        heading_pub = this->create_publisher<std_msgs::msg::Float32>(heading_topic_, 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // Initialize simulated_time_
        simulated_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        last_vel = simulated_time_;
        last_update = simulated_time_;

        // Initialize previous twist
        prev_twist_ = odom.twist.twist;

        RCLCPP_INFO(this->get_logger(), "Initialized mobile robot simulator with IMU in namespace '%s'", this->get_namespace());
    }

    // Destructor
    ~MobileRobotSimulator() {
        if (is_running) stop();
    }

    // Start the simulator
    void start() {
        auto duration = std::chrono::duration<double>(1.0 / publish_rate);
        loop_timer = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(duration),
            std::bind(&MobileRobotSimulator::update_loop, this));
        is_running = true;
        RCLCPP_INFO(this->get_logger(), "Started mobile robot simulator update loop, listening on '%s' topic", velocity_topic_.c_str());
    }

    // Stop the simulator
    void stop() {
        loop_timer->cancel();
        is_running = false;
        RCLCPP_INFO(this->get_logger(), "Stopped mobile robot simulator");
    }

private:
    // Update loop
    void update_loop() {
        // Increment simulated_time_
        simulated_time_ += rclcpp::Duration::from_seconds(1.0 / publish_rate);

        // Compute time difference since last update
        rclcpp::Time current_time = simulated_time_;
        rclcpp::Duration dt = current_time - last_update;
        double delta_t = dt.seconds();
        if (delta_t <= 0.0)
            delta_t = 1.0 / publish_rate;

        // Update odometry based on current velocities
        update_odom_from_vel(odom.twist.twist, dt);

        // Update timestamps
        pose.header.stamp = simulated_time_;
        odom.header.stamp = simulated_time_;

        // Publish GPS and heading
        gps_pub->publish(pose);
        heading_pub->publish(heading);

        // Publish Clock message
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock = simulated_time_;
        clock_pub_->publish(clock_msg);

        // Compute linear accelerations
        geometry_msgs::msg::Vector3 linear_accel;
        linear_accel.x = (odom.twist.twist.linear.x - prev_twist_.linear.x) / delta_t;
        linear_accel.y = (odom.twist.twist.linear.y - prev_twist_.linear.y) / delta_t;
        linear_accel.z = (odom.twist.twist.linear.z - prev_twist_.linear.z) / delta_t;

        // Prepare IMU message
        imu_msg_.header.stamp = current_time;
        imu_msg_.header.frame_id = "imu_link";

        // Orientation
        imu_msg_.orientation = odom.pose.pose.orientation;

        // Angular velocity
        imu_msg_.angular_velocity = odom.twist.twist.angular;

        // Linear acceleration
        imu_msg_.linear_acceleration = linear_accel;

        // Optionally, set covariance to zero
        for (int i = 0; i < 9; ++i) {
            imu_msg_.orientation_covariance[i] = 0.0;
            imu_msg_.angular_velocity_covariance[i] = 0.0;
            imu_msg_.linear_acceleration_covariance[i] = 0.0;
        }

        // Publish IMU message
        imu_pub_->publish(imu_msg_);

        // Update previous twist and last update time
        prev_twist_ = odom.twist.twist;
        last_update = current_time;
    }

    // Update odometry from velocity
    void update_odom_from_vel(const geometry_msgs::msg::Twist& vel_msg, const rclcpp::Duration& time_diff) {
        double dt = time_diff.seconds();

        // Compute changes in position and orientation
        double delta_x = (vel_msg.linear.x * std::cos(th) - vel_msg.linear.y * std::sin(th)) * dt;
        double delta_y = (vel_msg.linear.x * std::sin(th) + vel_msg.linear.y * std::cos(th)) * dt;
        double delta_th = vel_msg.angular.z * dt;
        double delta_z = vel_msg.linear.z * dt;

        // Update position and orientation
        odom.pose.pose.position.x += delta_x;
        odom.pose.pose.position.y += delta_y;
        odom.pose.pose.position.z += delta_z;
        th += delta_th;

        // Normalize the orientation angle
        th = std::atan2(std::sin(th), std::cos(th));

        // Update orientation in odometry
        tf2::Quaternion q;
        q.setRPY(0, 0, th);
        odom.pose.pose.orientation = tf2::toMsg(q);

        // The velocities are already updated in vel_callback
        // Do not overwrite them here
        // odom.twist.twist = vel_msg; // Remove this line

        // Convert odometry to GPS coordinates
        convert_odom_to_pose();

        // Update heading message
        heading.data = th * 180.0 / M_PI;
    }

    // Convert odometry to GPS pose
    void convert_odom_to_pose() {
        double lat, lon, alt;
        std::tie(lat, lon, alt) = loc::enu_to_gps(
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
            latitude_,
            longitude_,
            altitude_
        );
        pose.latitude = lat;
        pose.longitude = lon;
        pose.altitude = alt;
    }

    // Velocity callback
    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        measure_time = simulated_time_;
        rclcpp::Duration dt = measure_time - last_vel;

        if (dt > rclcpp::Duration::from_seconds(1.0 / publish_rate))
            dt = rclcpp::Duration::from_seconds(1.0 / publish_rate);

        last_vel = measure_time;

        // Update current velocities
        odom.twist.twist = *msg;
    }
};

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {{"use_sim_time", true}}
    );

    auto simulator = std::make_shared<MobileRobotSimulator>(options);
    simulator->start();

    rclcpp::spin(simulator);
    rclcpp::shutdown();
    return 0;
}
