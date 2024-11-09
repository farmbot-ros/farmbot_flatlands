#include "rclcpp/rclcpp.hpp"

// Message Types
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
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

class MobileRobotSimulator : public rclcpp::Node {
public:
    MobileRobotSimulator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("mobile_robot_simulator", options),
          is_running(false),
          th(0.0),
          message_received(false) {

        // Set the 'use_sim_time' parameter to true within the node
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        // Declare and get parameters
        this->declare_parameter<double>("publish_rate", 10.0);
        this->declare_parameter<std::string>("velocity_topic", "cmd_vel");
        this->declare_parameter<std::string>("odometry_topic", "pose");
        this->declare_parameter<std::string>("marker_topic", "robopose");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "base_link");

        // Declare and get initial pose parameters
        this->declare_parameter<double>("initial_x", 0.0);
        this->declare_parameter<double>("initial_y", 0.0);
        this->declare_parameter<double>("initial_theta", 0.0);

        this->get_parameter("publish_rate", publish_rate);
        this->get_parameter("velocity_topic", velocity_topic_);
        this->get_parameter("odometry_topic", odometry_topic_);
        this->get_parameter("marker_topic", marker_topic_);
        this->get_parameter("odom_frame", odom_frame);
        this->get_parameter("base_frame", base_frame);

        this->get_parameter("initial_x", initial_x_);
        this->get_parameter("initial_y", initial_y_);
        this->get_parameter("initial_theta", initial_theta_);

        // Create publishers and subscribers
        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic(), 50);
        vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            velocity_topic_, 5, std::bind(&MobileRobotSimulator::vel_callback, this, std::placeholders::_1));

        // Create marker publisher
        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);

        // Create clock publisher
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // Initialize odometry with the initial pose
        odom.header.frame_id = odom_frame;
        odom.child_frame_id = base_frame;
        odom.pose.pose.position.x = initial_x_;
        odom.pose.pose.position.y = initial_y_;
        odom.pose.pose.position.z = 0.0;
        th = initial_theta_;

        tf2::Quaternion q;
        q.setRPY(0, 0, th);
        odom.pose.pose.orientation = tf2::toMsg(q);

        // Initialize tf broadcaster
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

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
    // Helper functions to get topic names
    std::string velocity_topic() const {
        return velocity_topic_;
    }

    std::string odom_topic() const {
        return odometry_topic_;
    }

    std::string marker_topic() const {
        return marker_topic_;
    }

    void update_loop() {
        // Increment simulated_time_ by the loop duration
        simulated_time_ += rclcpp::Duration::from_seconds(1.0 / publish_rate);

        // Publish Clock message
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock = simulated_time_;
        clock_pub_->publish(clock_msg);

        last_update = simulated_time_;

        // Update the timestamp
        odom.header.stamp = last_update;
        odom_trans.header.stamp = last_update;

        // Publish odometry and transform
        odom_pub->publish(odom);
        get_tf_from_odom(odom);
        tf_broadcaster->sendTransform(odom_trans);
        message_received = false;

        // Publish the marker
        publish_marker();
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
    }

    void get_tf_from_odom(const nav_msgs::msg::Odometry& odom_msg) {
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header = odom_msg.header;
        odom_tf.child_frame_id = odom_msg.child_frame_id;
        odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
        odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
        odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
        odom_tf.transform.rotation = odom_msg.pose.pose.orientation;

        odom_trans = odom_tf;
    }

    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        measure_time = simulated_time_;
        rclcpp::Duration dt = measure_time - last_vel;
        last_vel = measure_time;
        if (dt >= rclcpp::Duration::from_seconds(0.5)) dt = rclcpp::Duration::from_seconds(0.1);
        message_received = true;
        update_odom_from_vel(*msg, dt);
    }

    void publish_marker() {
        // Set the frame ID and timestamp
        marker.header.frame_id = odom_frame;
        marker.header.stamp = simulated_time_;

        // Set the namespace and id for this marker
        marker.ns = this->get_namespace();  // Use the node's namespace
        marker.id = 0;

        // Set the marker type
        marker.type = visualization_msgs::msg::Marker::ARROW;

        // Set the marker action
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the pose of the marker (odometry pose)
        marker.pose = odom.pose.pose;

        // Set the scale of the marker
        marker.scale.x = 0.5;  // Length of the arrow
        marker.scale.y = 0.1;  // Width of the arrow shaft
        marker.scale.z = 0.1;  // Height of the arrow shaft

        // Set the color of the marker
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;  // Green color
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;  // Fully opaque

        // Set the lifetime of the marker
        marker.lifetime = rclcpp::Duration::from_seconds(0);  // 0 means marker persists until overwritten

        // Publish the marker
        marker_pub->publish(marker);
    }

    // Parameters
    double publish_rate;
    std::string velocity_topic_;
    std::string odometry_topic_;
    std::string marker_topic_;
    std::string odom_frame;
    std::string base_frame;

    // Initial pose parameters
    double initial_x_;
    double initial_y_;
    double initial_theta_;

    // Odometry and Transform Data
    nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::TransformStamped odom_trans;

    // Time Variables
    rclcpp::Time last_vel;
    rclcpp::Time last_update;
    rclcpp::Time measure_time;
    rclcpp::Time simulated_time_;  // Simulated time

    // State Variables
    bool is_running;
    double th;
    bool message_received;

    // ROS2 Interfaces
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;  // Marker publisher
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;      // Clock publisher
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::TimerBase::SharedPtr loop_timer;

    // Marker
    visualization_msgs::msg::Marker marker;  // Marker message
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
