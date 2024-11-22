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

// Other necessary includes
#include <rclcpp/rclcpp.hpp>

namespace sim {
    // Base class for obstacles
    class Obstacle {
        public:
            virtual ~Obstacle() = default;
            virtual bool contains(const geometry_msgs::msg::Point& point) const = 0;
            virtual std::string get_type() const = 0;
        };

    // Rectangle obstacle
    class RectangleObstacle : public Obstacle {
        private:
            double xmin_, xmax_, ymin_, ymax_;
        public:
            RectangleObstacle(double xmin, double xmax, double ymin, double ymax);
            bool contains(const geometry_msgs::msg::Point& point) const override;
            std::string get_type() const override { return "Rectangle"; }
        };
    // RectangleObstacle implementation
    inline RectangleObstacle::RectangleObstacle(double xmin, double xmax, double ymin, double ymax)
        : xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax) {}
    inline bool RectangleObstacle::contains(const geometry_msgs::msg::Point& point) const {
        return (point.x >= xmin_ && point.x <= xmax_ && point.y >= ymin_ && point.y <= ymax_);
    }

    // Circular obstacle
    class CircularObstacle : public Obstacle {
        private:
            double x_center_, y_center_, radius_;
        public:
            CircularObstacle(double x_center, double y_center, double radius);
            bool contains(const geometry_msgs::msg::Point& point) const override;
            std::string get_type() const override { return "Circle"; }
        };
    // CircularObstacle implementation
    inline CircularObstacle::CircularObstacle(double x_center, double y_center, double radius)
        : x_center_(x_center), y_center_(y_center), radius_(radius) {}
    inline bool CircularObstacle::contains(const geometry_msgs::msg::Point& point) const {
        double dx = point.x - x_center_;
        double dy = point.y - y_center_;
        return (dx*dx + dy*dy) <= (radius_*radius_);
    }

    // Zone class (could be similar to obstacles or have additional properties)
    class Zone {
        private:
            std::string name_;
            geometry_msgs::msg::Polygon area_;
        public:
            Zone(const std::string& name, const geometry_msgs::msg::Polygon& area);
            bool contains(const geometry_msgs::msg::Point& point) const;
            std::string get_name() const { return name_; }
        };
    // Zone implementation
    inline Zone::Zone(const std::string& name, const geometry_msgs::msg::Polygon& area)
        : name_(name), area_(area) {}
    // Simple point-in-polygon test (could be improved)
    inline bool Zone::contains(const geometry_msgs::msg::Point& point) const {
        // Implement point-in-polygon algorithm here (e.g., ray casting)
        return false; // Placeholder
    }

    // The Environment class
    class Environment {
        private:
            sensor_msgs::msg::NavSatFix datum_;
            std::vector<double> datum_param_;
            rclcpp::Node::SharedPtr node_;
            std::vector<std::shared_ptr<Obstacle>> obstacles_;
            std::vector<std::shared_ptr<Zone>> zones_;
            rclcpp::Logger logger_;
        public:
            Environment(const rclcpp::Node::SharedPtr& node);
            ~Environment();
            // Methods to add/remove obstacles and zones
            void add_obstacle(const std::shared_ptr<Obstacle>& obstacle);
            void add_zone(const std::shared_ptr<Zone>& zone);
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

        inline void Environment::add_obstacle(const std::shared_ptr<Obstacle>& obstacle) {
            obstacles_.push_back(obstacle);
            RCLCPP_INFO(logger_, "Added obstacle of type: %s", obstacle->get_type().c_str());
        }

        inline void Environment::add_zone(const std::shared_ptr<Zone>& zone) {
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
