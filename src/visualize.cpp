#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "farmbot_interfaces/msg/float32_stamped.hpp"
#include "farmbot_interfaces/srv/datum.hpp"
#include "farmbot_interfaces/srv/trigger.hpp"

class OdomNPath : public rclcpp::Node {
    private:
        std::string name;
        visualization_msgs::msg::Marker marker;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr enu_sub_;


    public:
        OdomNPath() : Node(
            "odom_n_path",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ){
            RCLCPP_INFO(this->get_logger(), "Startin vsualization node...");
            try {
                name = this->get_parameter("name").as_string();
            } catch (...) {
                name = "odom_n_path";
            }

            marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("loc/marker", 10);
            enu_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "loc/odom", 10, std::bind(&OdomNPath::base_transform, this, std::placeholders::_1)
            );
        }

    private:
        void base_transform(const nav_msgs::msg::Odometry::ConstSharedPtr& enu_odom) {
            // Set the frame ID and timestamp
            marker.header.frame_id = enu_odom->header.frame_id;
            marker.header.stamp = this->now();

            // Set the namespace and id for this marker
            marker.ns = this->get_namespace();  // Use the node's namespace
            marker.id = 0;

            // Set the marker type
            marker.type = visualization_msgs::msg::Marker::ARROW;

            // Set the marker action
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set the pose of the marker (odometry pose)
            marker.pose = enu_odom->pose.pose;

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
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto navfix = std::make_shared<OdomNPath>();
    rclcpp::spin(navfix);
    rclcpp::shutdown();
    return 0;
}
