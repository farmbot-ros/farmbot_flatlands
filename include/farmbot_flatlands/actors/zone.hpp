#ifndef ZONE_HPP
#define ZONE_HPP

#include <vector>
#include <memory>
#include <string>

// Include ROS geometry messages
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon.hpp"

// Include Boost geometry
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/detail/within/interface.hpp>

// For the Robot state
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "farmbot_flatlands/actors/actor.hpp"
#include "farmbot_flatlands/types.hpp"

// Other necessary includes
#include <rclcpp/rclcpp.hpp>

namespace sim {
    namespace actor{
        class Zone : Actor {
            private:
                std::string name_;
                Polygon polygon_;
            public:
                Zone(const std::string& name, const Polygon& polygon);
                bool contains(const Point& point) const override;
                bool contains(const geometry_msgs::msg::Point& point) const override;
                ActorType get_type() const override { return ActorType::ZONE; }
                std::string get_name() const { return name_; }
            };
        inline Zone::Zone(const std::string& name, const Polygon& polygon): name_(name), polygon_(polygon) {}
        inline bool Zone::contains(const Point& point) const {
            return boost::geometry::within(point, polygon_);
        }
        inline bool Zone::contains(const geometry_msgs::msg::Point& point) const {
            return contains(Point(point.x, point.y));
        }

    }// namespace actor
}// namespace sim

#endif // OBSTACLE_HPP
