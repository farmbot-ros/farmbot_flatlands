#include "rclcpp/rclcpp.hpp"

// Message Types
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rosgraph_msgs/msg/clock.hpp"  // Added for /clock

// TF2 Headers
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Standard Libraries
#include <functional>
#include <rclcpp/logging.hpp>
#include <string>
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tuple>


namespace loc {

    // Constants
    const double R = 6378137.0;                // Earth's radius in meters (equatorial radius)
    const double f = 1.0 / 298.257223563;      // Flattening factor
    const double e2 = 2 * f - f * f;           // Square of eccentricity

    // Function to convert GPS (lat, lon, alt) to ECEF coordinates
    std::tuple<double, double, double> gps_to_ecef(double latitude, double longitude, double altitude) {
        // Convert latitude and longitude to radians
        double cosLat = std::cos(latitude * M_PI / 180.0);
        double sinLat = std::sin(latitude * M_PI / 180.0);
        double cosLong = std::cos(longitude * M_PI / 180.0);
        double sinLong = std::sin(longitude * M_PI / 180.0);
        // Prime vertical radius of curvature
        double N = R / std::sqrt(1.0 - e2 * sinLat * sinLat);
        // Calculate ECEF coordinates
        double x = (N + altitude) * cosLat * cosLong;
        double y = (N + altitude) * cosLat * sinLong;
        double z = (N * (1 - e2) + altitude) * sinLat;
        // Return the ECEF coordinates
        return std::make_tuple(x, y, z);
    }

    // Function to convert ECEF coordinates to ENU (East, North, Up) with respect to a datum
    std::tuple<double, double, double> ecef_to_enu(std::tuple<double, double, double> ecef, std::tuple<double, double, double> datum) {
        double x, y, z;
        std::tie(x, y, z) = ecef;
        double latRef, longRef, altRef;
        std::tie(latRef, longRef, altRef) = datum;
        // Convert the reference latitude and longitude to radians
        double cosLatRef = std::cos(latRef * M_PI / 180.0);
        double sinLatRef = std::sin(latRef * M_PI / 180.0);
        double cosLongRef = std::cos(longRef * M_PI / 180.0);
        double sinLongRef = std::sin(longRef * M_PI / 180.0);
        // Prime vertical radius of curvature at the reference point
        double NRef = R / std::sqrt(1.0 - e2 * sinLatRef * sinLatRef);
        // Calculate the reference ECEF coordinates (datum)
        double x0 = (NRef + altRef) * cosLatRef * cosLongRef;
        double y0 = (NRef + altRef) * cosLatRef * sinLongRef;
        double z0 = (NRef * (1 - e2) + altRef) * sinLatRef;
        // Calculate the differences between the ECEF coordinates and the reference ECEF coordinates
        double dx = x - x0;
        double dy = y - y0;
        double dz = z - z0;
        // Calculate the ENU coordinates
        double xEast = -sinLongRef * dx + cosLongRef * dy;
        double yNorth = -cosLongRef * sinLatRef * dx - sinLatRef * sinLongRef * dy + cosLatRef * dz;
        double zUp = cosLatRef * cosLongRef * dx + cosLatRef * sinLongRef * dy + sinLatRef * dz;
        // Return the ENU coordinates
        return std::make_tuple(xEast, yNorth, zUp);
    }

    // Function to convert GPS (lat, lon, alt) to ENU (East, North, Up) with respect to a datum
    std::tuple<double, double, double> gps_to_enu(double latitude, double longitude, double altitude, double latRef, double longRef, double altRef) {
        // Convert GPS coordinates to ECEF
        std::tuple<double, double, double> ecef = gps_to_ecef(latitude, longitude, altitude);
        // Return the ENU coordinates
        return ecef_to_enu(ecef, std::make_tuple(latRef, longRef, altRef));
    }

    std::tuple<double, double, double> enu_to_ecef(std::tuple<double, double, double> enu, std::tuple<double, double, double> datum) {
        // Extract ENU and datum coordinates
        double xEast, yNorth, zUp;
        std::tie(xEast, yNorth, zUp) = enu;
        double latRef, longRef, altRef;
        std::tie(latRef, longRef, altRef) = datum;
        // Compute trigonometric values for the reference latitude and longitude
        double cosLatRef = std::cos(latRef * M_PI / 180);
        double sinLatRef = std::sin(latRef * M_PI / 180);
        double cosLongRef = std::cos(longRef * M_PI / 180);
        double sinLongRef = std::sin(longRef * M_PI / 180);
        // Compute reference ECEF coordinates for the datum
        double cRef = 1 / std::sqrt(cosLatRef * cosLatRef + (1 - f) * (1 - f) * sinLatRef * sinLatRef);
        double x0 = (R * cRef + altRef) * cosLatRef * cosLongRef;
        double y0 = (R * cRef + altRef) * cosLatRef * sinLongRef;
        double z0 = (R * cRef * (1 - e2) + altRef) * sinLatRef;
        // Reverse the ENU to ECEF transformation
        double x = x0 + (-sinLongRef * xEast) - (sinLatRef * cosLongRef * yNorth) + (cosLatRef * cosLongRef * zUp);
        double y = y0 + (cosLongRef * xEast) - (sinLatRef * sinLongRef * yNorth) + (cosLatRef * sinLongRef * zUp);
        double z = z0 + (cosLatRef * yNorth) + (sinLatRef * zUp);
        // Return the ECEF coordinates
        return std::make_tuple(x, y, z);
    }

    std::tuple<double, double, double> ecef_to_gps(double x, double y, double z) {
        const double e2 = f * (2 - f); // Square of eccentricity
        const double eps = 1e-12; // Convergence threshold
        double longitude = std::atan2(y, x) * 180 / M_PI;
        // Compute initial latitude and height guesses
        double p = std::sqrt(x * x + y * y);
        double latitude = std::atan2(z, p * (1 - e2));
        double N = R / std::sqrt(1 - e2 * std::sin(latitude) * std::sin(latitude)); // Prime vertical radius of curvature
        double altitude = p / std::cos(latitude) - N;
        double latOld;
        // Iterate to refine the latitude and height
        do {
            latOld = latitude;
            N = R / std::sqrt(1 - e2 * std::sin(latitude) * std::sin(latitude));
            altitude = p / std::cos(latitude) - N;
            latitude = std::atan2(z, p * (1 - e2 * N / (N + altitude)));
        } while (std::abs(latitude - latOld) > eps);
        // Return the GPS coordinates
        return std::make_tuple(latitude * 180 / M_PI, longitude, altitude);
    }

    std::tuple<double, double, double> enu_to_gps(double xEast, double yNorth, double zUp, double latRef, double longRef, double altRef) {
        // Convert ENU to ECEF
        std::tuple<double, double, double> ecef = enu_to_ecef(std::make_tuple(xEast, yNorth, zUp), std::make_tuple(latRef, longRef, altRef));
        // Convert ECEF to GPS
        return ecef_to_gps(std::get<0>(ecef), std::get<1>(ecef), std::get<2>(ecef));
    }
}

class MobileRobotSimulator : public rclcpp::Node {
    private:
        // Parameters
        double publish_rate;
        std::string velocity_topic_;
        double latitude_;
        double longitude_;
        double heading_;
        std::string odometry_topic_;
        std::string position_topic_;
        std::string reference_topic_;
        std::string heading_topic_;

        // ROS2 Messages
        geometry_msgs::msg::Twist vel;
        nav_msgs::msg::Odometry odom;
        sensor_msgs::msg::NavSatFix pose;
        sensor_msgs::msg::NavSatFix ref;
        std_msgs::msg::Float32 heading;

        // Time Variables
        rclcpp::Time last_vel;
        rclcpp::Time last_update;
        rclcpp::Time measure_time;
        rclcpp::Time simulated_time_;  // Simulated time

        // State Variables
        bool is_running;
        double th;

        // ROS2 Interfaces
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr ref_sub;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_pub;
        rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
        rclcpp::TimerBase::SharedPtr loop_timer;

public:
    MobileRobotSimulator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("mobile_simulator", options), is_running(false), th(0.0) {

        // Declare and get parameters
        this->declare_parameter<double>("publish_rate", 10.0);

        this->declare_parameter<std::string>("velocity_topic", "cmd_vel");
        this->declare_parameter<std::string>("odometry_topic", "odom");
        this->declare_parameter<std::string>("reference_topic", "ref");
        this->declare_parameter<std::string>("heading_topic", "heading");

        this->declare_parameter<std::string>("position_topic", "fix");
        this->declare_parameter<double>("latitude", 0.0);
        this->declare_parameter<double>("longitude", 0.0);
        this->declare_parameter<double>("heading", 0.0);

        this->get_parameter("publish_rate", publish_rate);

        this->get_parameter("velocity_topic", velocity_topic_);
        this->get_parameter("odometry_topic", odometry_topic_);
        this->get_parameter("reference_topic", reference_topic_);
        this->get_parameter("heading_topic", heading_topic_);

        this->get_parameter("position_topic", position_topic_);
        this->get_parameter("latitude", latitude_);
        this->get_parameter("longitude", longitude_);
        this->get_parameter("heading", heading_);

        // Create subscribers
        vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(velocity_topic_, 5, std::bind(&MobileRobotSimulator::vel_callback, this, std::placeholders::_1));
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_, 5, std::bind(&MobileRobotSimulator::odom_callback, this, std::placeholders::_1));
        ref_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(reference_topic_, 5, std::bind(&MobileRobotSimulator::ref_callback, this, std::placeholders::_1));

        // Fill the pose message
        pose.header.frame_id = "world";
        pose.latitude = latitude_;
        pose.longitude = longitude_;
        pose.altitude = 0.0;

        // Fill the heading message
        heading.data = heading_;


        // Create publishers
        gps_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>(position_topic_, 10);
        heading_pub = this->create_publisher<std_msgs::msg::Float32>(heading_topic_, 10);

        // Create clock publisher
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // Initialize simulated_time_
        simulated_time_ = this->get_clock()->now();
        last_vel = simulated_time_;

        RCLCPP_INFO(this->get_logger(), "Initialized mobile robot simulator in namespace '%s'", this->get_namespace());
    }

    ~MobileRobotSimulator() {
        if (is_running) stop();
    }

    void start() {
        auto duration = std::chrono::duration<double>(1.0 / publish_rate);
        loop_timer = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(duration),
            std::bind(&MobileRobotSimulator::update_loop, this));
        is_running = true;

        RCLCPP_INFO(this->get_logger(), "Started mobile robot simulator update loop, listening on '%s' topic", velocity_topic_.c_str());
    }

    void stop() {
        loop_timer->cancel();
        is_running = false;
        RCLCPP_INFO(this->get_logger(), "Stopped mobile robot simulator");
    }

private:
    void update_loop() {
        // Increment simulated_time_ by the loop duration
        simulated_time_ += rclcpp::Duration::from_seconds(1.0 / publish_rate);
        // Publish Clock message
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock = simulated_time_;
        clock_pub_->publish(clock_msg);
        last_update = simulated_time_;
        // Update the timestamp
        pose.header.stamp = simulated_time_;
        odom.header.stamp = simulated_time_;
        //publish
        gps_pub->publish(pose);
        heading_pub->publish(heading);
    }

    void update_odom_from_vel(const geometry_msgs::msg::Twist& vel, const rclcpp::Duration& time_diff) {
        double dt = time_diff.seconds();
        // Compute odometry
        double delta_x = (vel.linear.x * cos(th) - vel.linear.y * sin(th)) * dt;
        double delta_y = (vel.linear.x * sin(th) + vel.linear.y * cos(th)) * dt;
        double delta_th = vel.angular.z * dt;
        // Update odometry
        odom.pose.pose.position.x += delta_x;
        odom.pose.pose.position.y += delta_y;
        th += delta_th;
        // Normalize th to [-pi, pi]
        th = std::atan2(std::sin(th), std::cos(th));
        // Set orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, th);
        odom.pose.pose.orientation = tf2::toMsg(q);
        // Set velocity
        odom.twist.twist = vel;
        convert_odom_to_pose();
        heading.data = th;
    }

    void convert_odom_to_pose() {
        double lat, lon, alt;
        std::tie(lat, lon, alt) = loc::enu_to_gps(odom.pose.pose.position.x, odom.pose.pose.position.y, 0, latitude_, longitude_, 0);
        //update pose
        pose.latitude = lat;
        pose.longitude = lon;
    }


    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        measure_time = simulated_time_;
        rclcpp::Duration dt = measure_time - last_vel;
        last_vel = measure_time;
        if (dt >= rclcpp::Duration::from_seconds(0.5)) dt = rclcpp::Duration::from_seconds(0.1);
        update_odom_from_vel(*msg, dt);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        measure_time = simulated_time_;
        rclcpp::Duration dt = measure_time - last_update;
        last_update = measure_time;
        if (dt >= rclcpp::Duration::from_seconds(0.5)) dt = rclcpp::Duration::from_seconds(0.1);
        odom = *msg;
    }

    void ref_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        measure_time = simulated_time_;
        rclcpp::Duration dt = measure_time - last_update;
        last_update = measure_time;
        if (dt >= rclcpp::Duration::from_seconds(0.5)) dt = rclcpp::Duration::from_seconds(0.1);
        ref = *msg;
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create node options and set 'use_sim_time' parameter
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {{"use_sim_time", true}}  // Override use_sim_time to true
    );

    auto simulator = std::make_shared<MobileRobotSimulator>(options);
    simulator->start();

    rclcpp::spin(simulator);
    rclcpp::shutdown();
    return 0;
}
