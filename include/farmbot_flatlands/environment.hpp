#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <vector>
#include <memory>
#include <string>

// Include ROS geometry messages
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon.hpp"

// For the Robot state
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// Include actors
#include "farmbot_flatlands/actors/obstacle.hpp"
#include "farmbot_flatlands/actors/zone.hpp"

// Other necessary includes
#include <rclcpp/rclcpp.hpp>

namespace sim {
    // The Environment class
    class Environment {
        private:
            sensor_msgs::msg::NavSatFix datum_;
            std::vector<double> datum_param_;
            rclcpp::Node::SharedPtr node_;
            std::vector<std::shared_ptr<actor::CircularObstacle>> obstacles_;
            std::vector<std::shared_ptr<actor::Zone>> zones_;
            rclcpp::Logger logger_;
        public:
            Environment(const rclcpp::Node::SharedPtr& node);
            ~Environment();
            // Methods to add/remove obstacles and zones
            void add_obstacle(const std::shared_ptr<actor::CircularObstacle>& obstacle);
            void add_zone(const std::shared_ptr<actor::Zone>& zone);
            // Method to check if a point is in any obstacle
            bool is_collision(const geometry_msgs::msg::Point& point) const;
            // Method to check in which zones a point is
            std::vector<std::string> get_zones(const geometry_msgs::msg::Point& point) const;
            // Method to set/get the datum
            void set_datum(const sensor_msgs::msg::NavSatFix& datum) { datum_ = datum; }
            sensor_msgs::msg::NavSatFix get_datum() const { return datum_; }
        };
        // Environment implementation
        inline Environment::Environment(const rclcpp::Node::SharedPtr& node)
            : node_(node), logger_(node->get_logger()) {
                node_->declare_parameter<std::vector<double>>("datum", {0.0, 0.0, 0.0});
                node_->get_parameter("datum", datum_param_);

                datum_.header.frame_id = "gps";
                datum_.latitude = datum_param_[0];
                datum_.longitude = datum_param_[1];
                datum_.altitude = datum_param_[2];
                datum_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
                // RCLCPP_INFO(logger_, "Datum set to: %.6f, %.6f, %.2f", datum_.latitude, datum_.longitude, datum_.altitude);
            }
        inline Environment::~Environment() {}

        inline void Environment::add_obstacle(const std::shared_ptr<actor::CircularObstacle>& obstacle) {
            obstacles_.push_back(obstacle);
            RCLCPP_INFO(logger_, "Added obstacle");
        }

        inline void Environment::add_zone(const std::shared_ptr<actor::Zone>& zone) {
            zones_.push_back(zone);
            RCLCPP_INFO(logger_, "Added zone: %s", zone->get_name().c_str());
        }

        inline bool Environment::is_collision(const geometry_msgs::msg::Point& point) const {
            for (const auto& obstacle : obstacles_) {
                if (obstacle->contains(point)) {
                    return true;
                }
            }
            return false;
        }

        inline std::vector<std::string> Environment::get_zones(const geometry_msgs::msg::Point& point) const {
            std::vector<std::string> zone_names;
            for (const auto& zone : zones_) {
                if (zone->contains(point)) {
                    zone_names.push_back(zone->get_name());
                }
            }
            return zone_names;
        }
}

#endif // ENVIRONMENT_HPP
