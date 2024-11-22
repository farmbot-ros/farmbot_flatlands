#ifndef PLUGIN_HPP
#define PLUGIN_HPP

#include <string>
#include <tuple>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

namespace sim {
    class Plugin {
        public:
            virtual ~Plugin() = default;
            // virtual void init(...) = 0;
            virtual void tick2(const rclcpp::Time &current_time) = 0;
    };
}

#endif
