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
#include "muli/math.h"
#include "muli/polygon.h"
#include "muli/rigidbody.h"
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
            void set_datum(const std::vector<double>& datum_param);
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
            std::vector<std::pair<std::string, muli::RigidBody *>> robots_;
            std::vector<std::pair<std::string, muli::RigidBody *>> zones_;
            std::vector<std::pair<std::string, muli::RigidBody *>> obstacles_;

            rclcpp::Node::SharedPtr node_;
            std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> vel_subs_;
        public:
            WorldSettings settings;
            World(const WorldSettings& settings, const rclcpp::Node::SharedPtr& node);
            ~World();
            void step(double dt);
            muli::RigidBody* add_robot(float width, float height, std::string name);
            muli::RigidBody* get_robot(const std::string& name);
        };
        // World implementation
        inline World::World(const WorldSettings& settings, const rclcpp::Node::SharedPtr& node)
            : muli::World(settings), node_(node), settings(settings) {}
        inline World::~World() {}

        inline void World::step(double dt) {
            this->Step(dt);
        }

        inline muli::RigidBody* World::add_robot(float width, float height, std::string name) {
            muli::RigidBody* robot = this->CreateBox(width, height);
            robots_.push_back(std::make_pair(name, robot));
            return robot;
        }

        inline muli::RigidBody* World::get_robot(const std::string& name) {
            auto it = std::find_if(robots_.begin(), robots_.end(),
                [&name](const auto& pair) { return pair.first == name; });
            if (it != robots_.end()) {
                return it->second;
            }
            return nullptr;
        }
}

#endif // ENVIRONMENT_HPP
