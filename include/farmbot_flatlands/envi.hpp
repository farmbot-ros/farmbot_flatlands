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
#include "muli/polygon.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// Include actors
#include "muli/settings.h"
#include "muli/world.h"
#include "farmbot_flatlands/actors/obstacle.hpp"
#include "farmbot_flatlands/actors/zone.hpp"
#include "farmbot_flatlands/utils/types.hpp"

// World
#include "farmbot_flatlands/world.hpp"

// Other necessary includes
#include <rclcpp/rclcpp.hpp>

namespace sim {
    // The Environment class
    class Environment {
        private:
            std::shared_ptr<World> world_;
            rclcpp::Node::SharedPtr node_;
            std::vector<std::shared_ptr<actor::Obstacle>> obstacles_;
            std::vector<std::shared_ptr<actor::Zone>> zones_;
            rclcpp::Logger logger_;
        public:
            Environment(const rclcpp::Node::SharedPtr& node, std::shared_ptr<World> world);
            ~Environment();
            // Update
            void update(double delta_t, const rclcpp::Time & current_time);
            // Methods to add/remove obstacles and zones
            void add_obstacle(const std::shared_ptr<actor::Obstacle>& obstacle);
            void add_zone(const std::shared_ptr<actor::Zone>& zone);
            // Method to check if a point is in any obstacle
            bool is_collision(const geometry_msgs::msg::Point& point) const;
            // Method to check in which zones a point is
            std::vector<std::string> get_zones(const geometry_msgs::msg::Point& point) const;
            // Method to set/get the datum
        };
        // Environment implementation
        inline Environment::Environment(const rclcpp::Node::SharedPtr& node, std::shared_ptr<World> world)
            : world_(world), node_(node), logger_(node->get_logger()) {}
        inline Environment::~Environment() {}

        inline void Environment::update(double dt, const rclcpp::Time & current_time) {}

        inline void Environment::add_obstacle(const std::shared_ptr<actor::Obstacle>& obstacle) {
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
