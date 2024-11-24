#ifndef WORLD_HPP
#define WORLD_HPP

#include <sensor_msgs/msg/detail/nav_sat_fix__struct.hpp>
#include <vector>
#include <memory>
#include <string>

// Include ROS geometry messages
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon.hpp"

// For the Robot state
#include "muli/settings.h"
#include "muli/world.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// Include actors
// #include "farmbot_flatlands/robot.hpp"
// #include "farmbot_flatlands/envi.hpp"
//
// Other necessary includes
#include <rclcpp/rclcpp.hpp>

// Include Muli

namespace sim {

    class WorldSettings : public muli::WorldSettings {
        private:
            sensor_msgs::msg::NavSatFix datum_;
        public:
            int num_obstacles_;
            int num_robots_;
        public:
            WorldSettings() : muli::WorldSettings() {}
            // setters
            void set_datum(const std::vector<double>& datum_param);
            // getters
            sensor_msgs::msg::NavSatFix get_datum() { return datum_; }
    };

    void WorldSettings::set_datum(const std::vector<double>& datum_param) {
        datum_.header.frame_id = "gps";
        datum_.latitude = datum_param[0];
        datum_.longitude = datum_param[1];
        datum_.altitude = datum_param[2];
    }

    // The Environment class
    class World : public muli::World {
        private:
            rclcpp::Node::SharedPtr node_;
            // std::vector<std::shared_ptr<Robot>> robots_;
            std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> vel_subs_;
        public:
            WorldSettings settings;
            World(const WorldSettings& settings, const rclcpp::Node::SharedPtr& node);
            ~World();
            void create_world();
            void step(double dt);
        };
        // World implementation
        inline World::World(const WorldSettings& settings, const rclcpp::Node::SharedPtr& node)
            : muli::World(settings), node_(node), settings(settings) {}
        inline World::~World() {}

        inline void World::create_world(){
            for (int i = 0; i < settings.num_robots_; i++){
                std::string robot_name = "robot" + std::to_string(i);
                // robots_.push_back(std::make_shared<Robot>(node_, world_));
                // robots_[i]->init(robot_name);
                // vel_subs_.push_back(node_->create_subscription<geometry_msgs::msg::Twist>(
                    // robot_name+"/cmd_vel", 10,
                    // [this, i](geometry_msgs::msg::Twist::SharedPtr msg) { robots_[i]->set_twist(*msg); }));
            }

        }
}

#endif // ENVIRONMENT_HPP
