#ifndef ACTOR_HPP
#define ACTOR_HPP

#include <string>
#include <tuple>
#include <cmath>
#include <any>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "farmbot_flatlands/utils/types.hpp"

namespace sim {

    // actor types
    enum class ActorType {
        AGENT,
        OBSTACLE,
        ZONE
    };

    class Actor {
        public:
            virtual ~Actor() = default;
            virtual bool contains(const Point& point) const = 0;
            virtual bool contains(const geometry_msgs::msg::Point& point) const = 0;
            virtual ActorType get_type() const = 0;
    };
}

#endif
