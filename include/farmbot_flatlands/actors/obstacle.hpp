
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
// Include Boost geometry
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/detail/within/interface.hpp>

#include "farmbot_flatlands/actors/actor.hpp"
#include "farmbot_flatlands/types.hpp"

// Other necessary includes
#include <rclcpp/rclcpp.hpp>

namespace sim {
    namespace actor{

        // Rectangle obstacle
        class Obstacle : public Actor {
            private:
                Polygon polygon_;
            public:
                Obstacle(const Polygon& polygon);
                bool contains(const Point& point) const override;
                bool contains(const geometry_msgs::msg::Point& point) const override;
                ActorType get_type() const override { return ActorType::OBSTACLE; }
            };
        // Obstacle implementation
        inline Obstacle::Obstacle(const Polygon& polygon) : polygon_(polygon) {}
        inline bool Obstacle::contains(const Point& point) const {
            return boost::geometry::within(point, polygon_);
        }
        inline bool Obstacle::contains(const geometry_msgs::msg::Point& point) const {
            return contains(Point(point.x, point.y));
        }

    }// namespace actor
}// namespace sim

#endif // ENVIRONMENT_HPP
