
#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

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

#include "farmbot_flatlands/actors/actor.hpp"

// Other necessary includes
#include <rclcpp/rclcpp.hpp>

namespace sim {
    namespace actor{

        // Rectangle obstacle
        class RectangleObstacle : public Actor {
            private:
                double xmin_, xmax_, ymin_, ymax_;
            public:
                RectangleObstacle(double xmin, double xmax, double ymin, double ymax);
                bool contains(const geometry_msgs::msg::Point& point) const override;
                ActorType get_type() const override { return ActorType::OBSTACLE; }
            };
        // RectangleObstacle implementation
        inline RectangleObstacle::RectangleObstacle(double xmin, double xmax, double ymin, double ymax)
            : xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax) {}
        inline bool RectangleObstacle::contains(const geometry_msgs::msg::Point& point) const {
            return (point.x >= xmin_ && point.x <= xmax_ && point.y >= ymin_ && point.y <= ymax_);
        }

        // Circular obstacle
        class CircularObstacle : public Actor {
            private:
                double x_center_, y_center_, radius_;
            public:
                CircularObstacle(double x_center, double y_center, double radius);
                bool contains(const geometry_msgs::msg::Point& point) const override;
                ActorType get_type() const override { return ActorType::OBSTACLE; }
            };
        // CircularObstacle implementation
        inline CircularObstacle::CircularObstacle(double x_center, double y_center, double radius)
            : x_center_(x_center), y_center_(y_center), radius_(radius) {}
        inline bool CircularObstacle::contains(const geometry_msgs::msg::Point& point) const {
            double dx = point.x - x_center_;
            double dy = point.y - y_center_;
            return (dx*dx + dy*dy) <= (radius_*radius_);
        }

    }// namespace actor
}// namespace sim

#endif // ENVIRONMENT_HPP
