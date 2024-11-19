
#ifndef GPS_PLUGIN_HPP
#define GPS_PLUGIN_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"

#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/detail/nav_sat_fix__struct.hpp>
#include <string>
#include <tuple>
#include <cmath>

namespace sim {
    namespace plugins {
        namespace loc {
            const double R = 6378137.0;                // Earth's radius in meters (equatorial radius)
            const double f = 1.0 / 298.257223563;      // Flattening factor of the Earth
            const double e2 = 2 * f - f * f;           // Square of the Earth's eccentricity

            // Function to convert ENU coordinates to ECEF coordinates relative to a datum
            inline std::tuple<double, double, double> enu_to_ecef(std::tuple<double, double, double> enu, std::tuple<double, double, double> datum) {
                double xEast, yNorth, zUp;
                std::tie(xEast, yNorth, zUp) = enu;
                double latRef, longRef, altRef;
                std::tie(latRef, longRef, altRef) = datum;
                double latRef_rad = latRef * M_PI / 180.0;
                double longRef_rad = longRef * M_PI / 180.0;
                double cosLatRef = std::cos(latRef_rad);
                double sinLatRef = std::sin(latRef_rad);
                double cosLongRef = std::cos(longRef_rad);
                double sinLongRef = std::sin(longRef_rad);
                double NRef = R / std::sqrt(1.0 - e2 * sinLatRef * sinLatRef);
                double x0 = (NRef + altRef) * cosLatRef * cosLongRef;
                double y0 = (NRef + altRef) * cosLatRef * sinLongRef;
                double z0 = (NRef * (1 - e2) + altRef) * sinLatRef;
                double dx = -sinLongRef * xEast - sinLatRef * cosLongRef * yNorth + cosLatRef * cosLongRef * zUp;
                double dy = cosLongRef * xEast - sinLatRef * sinLongRef * yNorth + cosLatRef * sinLongRef * zUp;
                double dz = cosLatRef * yNorth + sinLatRef * zUp;
                double x = x0 + dx;
                double y = y0 + dy;
                double z = z0 + dz;
                return std::make_tuple(x, y, z);
            }

            // Function to convert ECEF coordinates to GPS coordinates (latitude, longitude, altitude)
            inline std::tuple<double, double, double> ecef_to_gps(double x, double y, double z) {
                const double eps = 1e-12; // Convergence threshold
                double longitude = std::atan2(y, x) * 180.0 / M_PI;
                double p = std::sqrt(x * x + y * y);
                double theta = std::atan2(z * R, p * (1 - f) * R);
                double sinTheta = std::sin(theta);
                double cosTheta = std::cos(theta);
                double latitude = std::atan2(z + e2 * (1 - f) * R * sinTheta * sinTheta * sinTheta,
                                            p - e2 * R * cosTheta * cosTheta * cosTheta);
                double N = R / std::sqrt(1.0 - e2 * std::sin(latitude) * std::sin(latitude));
                double altitude = p / std::cos(latitude) - N;
                double latOld;
                do {
                    latOld = latitude;
                    N = R / std::sqrt(1.0 - e2 * std::sin(latitude) * std::sin(latitude));
                    altitude = p / std::cos(latitude) - N;
                    latitude = std::atan2(z + e2 * N * std::sin(latitude), p);
                } while (std::abs(latitude - latOld) > eps);
                return std::make_tuple(latitude * 180.0 / M_PI, longitude, altitude);
            }

            // Function to convert ENU coordinates to GPS coordinates relative to a datum
            inline std::tuple<double, double, double> enu_to_gps(double xEast, double yNorth, double zUp, double latRef, double longRef, double altRef) {
                auto ecef = enu_to_ecef(std::make_tuple(xEast, yNorth, zUp), std::make_tuple(latRef, longRef, altRef));
                return ecef_to_gps(std::get<0>(ecef), std::get<1>(ecef), std::get<2>(ecef));
            }

        } // namespace loc

        class GPSPlugin {
            private:
                rclcpp::Node::SharedPtr node_;
                std::string topic_;
                sensor_msgs::msg::NavSatFix fix_;
                nav_msgs::msg::Odometry odom_;
                rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
                rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_pub;

                // Reference coordinates for ENU
                double lat_ref_;
                double lon_ref_;
                double alt_ref_;

                // Last time and fix
                rclcpp::Time last_time_;
                sensor_msgs::msg::NavSatFix last_fix_;

            public:
                GPSPlugin() = default;
                GPSPlugin(rclcpp::Node::SharedPtr node, std::string topic, const sensor_msgs::msg::NavSatFix & fix):
                    node_(node),
                    topic_(topic),
                    fix_(fix),
                    lat_ref_(fix.latitude),
                    lon_ref_(fix.longitude),
                    alt_ref_(fix.altitude),
                    last_time_(node_->now())
                {
                    RCUTILS_LOG_INFO("GPS Plugin initialized");
                    //create publisher
                    gps_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>(topic_, 10);
                }

                void tick(const rclcpp::Time & current_time, const nav_msgs::msg::Odometry & odom) {
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

                    // Update last time
                    last_time_ = current_time;

                    // RCLCPP_INFO(node_->get_logger(), "GPS - Lat: %f, Lon: %f, Alt: %f", lat, lon, alt);
                    //print lat lon alt references
                    // RCLCPP_INFO(node_->get_logger(), "GPS - Lat Ref: %f, Lon Ref: %f, Alt Ref: %f", lat_ref_, lon_ref_, alt_ref_);
                    gps_pub_->publish(fix_);
                }
        };

    } // namespace plugins
} // namespace mobile_robot_simulator

#endif // GPS_PLUGIN_HPP
