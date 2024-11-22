#ifndef GPS_PLUGIN_HPP
#define GPS_PLUGIN_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"

#include "farmbot_flatlands/plugins/plugin.hpp"
#include "farmbot_flatlands/cords.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/nav_sat_fix__struct.hpp>
#include <string>
#include <tuple>
#include <cmath>

namespace sim {
    namespace plugins {
        class GPSPlugin : public Plugin {
            private:
                rclcpp::Node::SharedPtr node_;
                std::string topic_;
                sensor_msgs::msg::NavSatFix fix_;
                nav_msgs::msg::Odometry odom_;
                rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
                rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_pub_;
                // Reference coordinates for ENU
                double lat_ref_;
                double lon_ref_;
                double alt_ref_;
                // Last time and fix
                rclcpp::Time last_time_;
                sensor_msgs::msg::NavSatFix last_fix_;
                // Previous position for heading calculation
                double prev_x_;
                double prev_y_;
                bool first_tick_;
                // Heading variables
                double heading_deg;
                double heading_rad;
            public:
                // GPSPlugin() = default;
                void init(const std::vector<std::any>& args) override { return; }

                GPSPlugin(rclcpp::Node::SharedPtr node, std::string topic, const sensor_msgs::msg::NavSatFix & fix):
                    node_(node),
                    topic_(topic),
                    fix_(fix),
                    lat_ref_(fix.latitude),
                    lon_ref_(fix.longitude),
                    alt_ref_(fix.altitude),
                    last_time_(node_->now()),
                    prev_x_(0.0),
                    prev_y_(0.0),
                    first_tick_(true)
                {
                    RCLCPP_INFO(node_->get_logger(), "GPS Plugin initialized on topic: %s", topic.c_str());
                    // Create publishers
                    gps_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>(topic_ + "/fix", 10);
                    heading_pub_ = node_->create_publisher<std_msgs::msg::Float32>(topic + "/heading", 10);
                    heading_deg = 0.0;
                    heading_rad = 0.0;
                }

                void tick(const rclcpp::Time & current_time, const nav_msgs::msg::Odometry & odom) override {
                    odom_ = odom;
                    // Convert ENU coordinates to GPS
                    double lat, lon, alt;
                    std::tie(lat, lon, alt) = loc::enu_to_gps(
                        odom_.pose.pose.position.x,
                        odom_.pose.pose.position.y,
                        odom_.pose.pose.position.z,
                        lat_ref_,
                        lon_ref_,
                        alt_ref_
                    );

                    // Update GPS pose
                    fix_.latitude = lat;
                    fix_.longitude = lon;
                    fix_.altitude = alt;
                    fix_.header.stamp = current_time;

                    // Publish GPS data
                    gps_pub_->publish(fix_);

                    // Calculate and publish heading
                    if (first_tick_) {
                        // Initialize previous positions
                        prev_x_ = odom_.pose.pose.position.x;
                        prev_y_ = odom_.pose.pose.position.y;
                        first_tick_ = false;
                    } else {
                        double delta_x = odom_.pose.pose.position.x - prev_x_;
                        double delta_y = odom_.pose.pose.position.y - prev_y_;

                        double distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);
                        if (distance > 0.01) {
                            heading_rad = std::atan2(delta_y, delta_x);
                            heading_deg = heading_rad * 180.0 / M_PI;
                        }

                        // Normalize heading to [0, 360)
                        if (heading_deg < 0) {
                            heading_deg += 360.0;
                        }

                        // Create and publish heading message
                        std_msgs::msg::Float32 heading_msg;
                        heading_msg.data = heading_deg;
                        heading_pub_->publish(heading_msg);

                        // Update previous positions
                        prev_x_ = odom_.pose.pose.position.x;
                        prev_y_ = odom_.pose.pose.position.y;
                    }

                    // Update last time
                    last_time_ = current_time;
                }
        };

    } // namespace plugins
} // namespace sim

#endif // GPS_PLUGIN_HPP
