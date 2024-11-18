#include "rclcpp/rclcpp.hpp"  // Include the main ROS2 C++ client library

// Message Types
#include "geometry_msgs/msg/twist.hpp"             // Include Twist message for velocity commands
#include "nav_msgs/msg/odometry.hpp"               // Include Odometry message for robot state
#include "sensor_msgs/msg/nav_sat_fix.hpp"         // Include NavSatFix message for GPS data
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

    // Function to convert GPS coordinates (latitude, longitude, altitude) to ECEF (Earth-Centered, Earth-Fixed) coordinates
    std::tuple<double, double, double> gps_to_ecef(double latitude, double longitude, double altitude) {
        // Convert latitude and longitude from degrees to radians
        double lat_rad = latitude * M_PI / 180.0;
        double lon_rad = longitude * M_PI / 180.0;
        double cosLat = std::cos(lat_rad);      // Cosine of latitude
        double sinLat = std::sin(lat_rad);      // Sine of latitude
        double cosLong = std::cos(lon_rad);     // Cosine of longitude
        double sinLong = std::sin(lon_rad);     // Sine of longitude

        // Calculate the prime vertical radius of curvature
        double N = R / std::sqrt(1.0 - e2 * sinLat * sinLat);

        // Calculate ECEF coordinates
        double x = (N + altitude) * cosLat * cosLong;
        double y = (N + altitude) * cosLat * sinLong;
        double z = (N * (1 - e2) + altitude) * sinLat;

        // Return the ECEF coordinates as a tuple
        return std::make_tuple(x, y, z);
    }

    // Function to convert ECEF coordinates to ENU (East, North, Up) coordinates relative to a datum
    std::tuple<double, double, double> ecef_to_enu(std::tuple<double, double, double> ecef, std::tuple<double, double, double> datum) {
        double x, y, z;
        std::tie(x, y, z) = ecef;  // Unpack ECEF coordinates
        double latRef, longRef, altRef;
        std::tie(latRef, longRef, altRef) = datum;  // Unpack datum coordinates

        // Convert reference latitude and longitude from degrees to radians
        double latRef_rad = latRef * M_PI / 180.0;
        double longRef_rad = longRef * M_PI / 180.0;
        double cosLatRef = std::cos(latRef_rad);
        double sinLatRef = std::sin(latRef_rad);
        double cosLongRef = std::cos(longRef_rad);
        double sinLongRef = std::sin(longRef_rad);

        // Calculate the prime vertical radius of curvature at the reference point
        double NRef = R / std::sqrt(1.0 - e2 * sinLatRef * sinLatRef);

        // Calculate the reference ECEF coordinates based on the datum
        double x0 = (NRef + altRef) * cosLatRef * cosLongRef;
        double y0 = (NRef + altRef) * cosLatRef * sinLongRef;
        double z0 = (NRef * (1 - e2) + altRef) * sinLatRef;

        // Compute differences between the target ECEF coordinates and the reference ECEF coordinates
        double dx = x - x0;
        double dy = y - y0;
        double dz = z - z0;

        // Calculate ENU coordinates using rotation matrices
        double xEast = -sinLongRef * dx + cosLongRef * dy;
        double yNorth = -cosLongRef * sinLatRef * dx - sinLatRef * sinLongRef * dy + cosLatRef * dz;
        double zUp = cosLatRef * cosLongRef * dx + cosLatRef * sinLongRef * dy + sinLatRef * dz;

        // Return the ENU coordinates as a tuple
        return std::make_tuple(xEast, yNorth, zUp);
    }

    // Function to convert GPS coordinates to ENU coordinates relative to a datum
    std::tuple<double, double, double> gps_to_enu(double latitude, double longitude, double altitude, double latRef, double longRef, double altRef) {
        // First, convert GPS to ECEF coordinates
        std::tuple<double, double, double> ecef = gps_to_ecef(latitude, longitude, altitude);
        // Then, convert ECEF to ENU coordinates using the datum
        return ecef_to_enu(ecef, std::make_tuple(latRef, longRef, altRef));
    }

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
    double publish_rate;                        // Rate at which data is published
    std::string velocity_topic_;                // Topic name for velocity commands
    double latitude_;                           // Initial latitude of the robot
    double longitude_;                          // Initial longitude of the robot
    double altitude_;                           // Initial altitude of the robot
    double heading_;                            // Initial heading (orientation) of the robot in radians
    std::string position_topic_;                // Topic name for GPS position data
    std::string heading_topic_;                 // Topic name for heading data

    // ROS2 Messages
    geometry_msgs::msg::Twist vel;              // Twist message to store velocity commands
    nav_msgs::msg::Odometry odom;               // Odometry message to store robot state
    sensor_msgs::msg::NavSatFix pose;           // NavSatFix message to store current GPS pose
    std_msgs::msg::Float32 heading;             // Float32 message to store heading information in degrees

    // Time Variables
    rclcpp::Time last_vel;                      // Timestamp of the last velocity command
    rclcpp::Time last_update;                   // Timestamp of the last update
    rclcpp::Time measure_time;                  // Current measurement time
    rclcpp::Time simulated_time_;               // Simulated time for the node

    // State Variables
    bool is_running;                            // Flag to indicate if the simulator is running
    double th;                                  // Current orientation angle (theta) in radians

    // ROS2 Interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub;       // Subscriber for velocity commands
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub;        // Publisher for GPS position data
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_pub;         // Publisher for heading data
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;       // Publisher for simulated clock
    rclcpp::TimerBase::SharedPtr loop_timer;                                  // Timer for the update loop

public:
    // Constructor for the MobileRobotSimulator class
    MobileRobotSimulator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("mobile_simulator", options), is_running(false), th(0.0) {

        // Declare and get parameters with default values
        this->declare_parameter<double>("publish_rate", 10.0);                // Publishing rate in Hz

        this->declare_parameter<std::string>("velocity_topic", "cmd_vel");    // Default velocity topic
        this->declare_parameter<std::string>("heading_topic", "heading");     // Default heading topic

        this->declare_parameter<std::string>("position_topic", "fix");        // Default GPS position topic
        this->declare_parameter<double>("latitude", 0.0);                     // Default latitude
        this->declare_parameter<double>("longitude", 0.0);                    // Default longitude
        this->declare_parameter<double>("altitude", 0.0);                     // Default altitude
        this->declare_parameter<double>("heading", 0.0);                      // Default heading in degrees

        // Retrieve parameter values
        this->get_parameter("publish_rate", publish_rate);

        this->get_parameter("velocity_topic", velocity_topic_);
        this->get_parameter("heading_topic", heading_topic_);

        this->get_parameter("position_topic", position_topic_);
        this->get_parameter("latitude", latitude_);
        this->get_parameter("longitude", longitude_);
        this->get_parameter("altitude", altitude_);
        this->get_parameter("heading", heading_);

        // Convert initial heading from degrees to radians
        th = heading_ * M_PI / 180.0; // Initialize th with the initial heading in radians

        // Initialize odometry message
        odom.header.frame_id = "world";
        odom.pose.pose.position.x = 0.0;
        odom.pose.pose.position.y = 0.0;
        odom.pose.pose.position.z = 0.0; // Start at zero altitude in ENU coordinates

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
        vel.linear.x = 0.0;
        vel.linear.y = 0.0;
        vel.linear.z = 0.0;
        vel.angular.x = 0.0;
        vel.angular.y = 0.0;
        vel.angular.z = 0.0;

        // Initialize the pose message with the frame ID and initial GPS coordinates
        pose.header.frame_id = "world";    // Set the coordinate frame to "world"
        pose.latitude = latitude_;         // Set initial latitude
        pose.longitude = longitude_;       // Set initial longitude
        pose.altitude = altitude_;         // Set initial altitude

        // Initialize the heading message with the initial heading value in degrees
        heading.data = heading_; // Heading in degrees

        // Create subscriber for velocity commands
        vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            velocity_topic_, 5,
            std::bind(&MobileRobotSimulator::vel_callback, this, std::placeholders::_1));

        // Create publishers for GPS position and heading
        gps_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>(position_topic_, 10);
        heading_pub = this->create_publisher<std_msgs::msg::Float32>(heading_topic_, 10);

        // Create a publisher for the simulated clock
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // Initialize simulated_time_ with zero time
        simulated_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

        last_vel = simulated_time_;    // Initialize last_vel with the simulated time
        last_update = simulated_time_; // Initialize last_update with the simulated time

        // Log an informational message indicating successful initialization
        RCLCPP_INFO(this->get_logger(), "Initialized mobile robot simulator in namespace '%s'", this->get_namespace());
    }

    // Destructor to ensure the simulator stops if it's running
    ~MobileRobotSimulator() {
        if (is_running) stop(); // Call stop() if the simulator is running
    }

    // Function to start the simulator's update loop
    void start() {
        // Calculate the duration between updates based on the publish rate
        auto duration = std::chrono::duration<double>(1.0 / publish_rate);
        // Create a wall timer that calls update_loop at the specified publish rate
        loop_timer = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(duration),
            std::bind(&MobileRobotSimulator::update_loop, this));
        is_running = true; // Set the running flag to true

        // Log an informational message indicating the simulator has started
        RCLCPP_INFO(this->get_logger(), "Started mobile robot simulator update loop, listening on '%s' topic", velocity_topic_.c_str());
    }

    // Function to stop the simulator's update loop
    void stop() {
        loop_timer->cancel(); // Cancel the timer
        is_running = false;    // Set the running flag to false
        // Log an informational message indicating the simulator has stopped
        RCLCPP_INFO(this->get_logger(), "Stopped mobile robot simulator");
    }

private:
    // Function called periodically by the timer to update and publish data
    void update_loop() {
        // Increment simulated_time_ by the loop duration
        simulated_time_ += rclcpp::Duration::from_seconds(1.0 / publish_rate);

        // Update the timestamp in messages
        pose.header.stamp = simulated_time_;
        odom.header.stamp = simulated_time_;

        // Publish GPS and heading
        gps_pub->publish(pose);
        heading_pub->publish(heading);

        // Publish Clock message
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock = simulated_time_;
        clock_pub_->publish(clock_msg);

        //print lat, lon, alt
        // RCLCPP_INFO(this->get_logger(), "Lat: %f, Lon: %f, Alt: %f", pose.latitude, pose.longitude, pose.altitude);
    }

    // Function to update odometry based on incoming velocity commands
    void update_odom_from_vel(const geometry_msgs::msg::Twist& vel_msg, const rclcpp::Duration& time_diff) {
        double dt = time_diff.seconds();   // Convert time difference to seconds

        // Compute changes in position and orientation based on velocity and time
        double delta_x = (vel_msg.linear.x * std::cos(th) - vel_msg.linear.y * std::sin(th)) * dt;
        double delta_y = (vel_msg.linear.x * std::sin(th) + vel_msg.linear.y * std::cos(th)) * dt;
        double delta_th = vel_msg.angular.z * dt;
        double delta_z = vel_msg.linear.z * dt; // Compute vertical displacement

        // Update the odometry pose with the computed deltas
        odom.pose.pose.position.x += delta_x;
        odom.pose.pose.position.y += delta_y;
        odom.pose.pose.position.z += delta_z; // Update altitude based on vertical movement
        th += delta_th; // Update the orientation angle in radians

        // Normalize the orientation angle to the range [-pi, pi]
        th = std::atan2(std::sin(th), std::cos(th));

        // Create a quaternion from the updated orientation angle
        tf2::Quaternion q;
        q.setRPY(0, 0, th); // Roll and pitch are zero; yaw is th
        odom.pose.pose.orientation = tf2::toMsg(q); // Set the orientation in the odometry message

        // Set the velocity in the odometry message
        odom.twist.twist = vel_msg;

        // Convert the updated odometry to GPS coordinates
        convert_odom_to_pose();

        // Update the heading message with the current orientation angle converted to degrees
        heading.data = th * 180.0 / M_PI; // Convert radians to degrees
    }

    // Function to convert odometry data to GPS pose
    void convert_odom_to_pose() {
        double lat, lon, alt;
        // Convert ENU coordinates from odometry to GPS coordinates using the initial latitude and longitude as datum
        std::tie(lat, lon, alt) = loc::enu_to_gps(
            odom.pose.pose.position.x,    // East coordinate
            odom.pose.pose.position.y,    // North coordinate
            odom.pose.pose.position.z,    // Up coordinate (now includes altitude changes)
            latitude_,                    // Reference latitude
            longitude_,                   // Reference longitude
            altitude_                     // Reference altitude
        );
        // Update the pose message with the new latitude, longitude, and altitude
        pose.latitude = lat;
        pose.longitude = lon;
        pose.altitude = alt;
    }

    // Callback function for incoming velocity commands
    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        measure_time = simulated_time_; // Set the measurement time to the current simulated time
        rclcpp::Duration dt = measure_time - last_vel; // Calculate the time difference since the last velocity command

        // If the time difference is too large, cap it to prevent large jumps
        if (dt > rclcpp::Duration::from_seconds(1.0 / publish_rate))
            dt = rclcpp::Duration::from_seconds(1.0 / publish_rate);

        last_vel = measure_time; // Update the last_vel timestamp

        // Update odometry based on the received velocity message and time difference
        update_odom_from_vel(*msg, dt);
    }
};

// Main function to initialize and run the ROS2 node
int main(int argc, char** argv) {
    rclcpp::init(argc, argv); // Initialize the ROS2 client library

    // Create node options and set the 'use_sim_time' parameter to true
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {{"use_sim_time", true}}  // Override the 'use_sim_time' parameter to enable simulated time
    );

    // Create an instance of the MobileRobotSimulator node with the specified options
    auto simulator = std::make_shared<MobileRobotSimulator>(options);
    simulator->start(); // Start the simulator's update loop

    rclcpp::spin(simulator); // Spin the node to process callbacks
    rclcpp::shutdown();      // Shutdown the ROS2 client library
    return 0;                // Exit the program
}
