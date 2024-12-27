#include <cstdint>
#include <fstream>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "farmbot_flatlands/sim.hpp"
#include "farmbot_flatlands/robot.hpp"

#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#include "ament_index_cpp/get_package_share_directory.hpp"
namespace aix = ament_index_cpp;
#include <spdlog/spdlog.h>
namespace echo = spdlog;


class ConfigParser {
private:
    std::vector<robo::RobotConfig> robots_;

public:
    ConfigParser(const std::string& filename) {
        std::string pcg = aix::get_package_share_directory("farmbot_flatlands");
        std::string path = pcg + "/config/" + filename;
        echo::info("Reading config file: {}", path);

        // Parse YAML file
        YAML::Node config = YAML::LoadFile(path);

        // Extract robots from YAML
        // datum: [51.987305, 5.663625, 53.801823] # Wageningen
        const auto& datum = config["global"]["ros__parameters"]["datum"];
        const auto& robots_config = config["global"]["ros__parameters"]["robots"];
        for (std::size_t i = 0; i < robots_config.size(); ++i) {
            const auto& robot_yaml = robots_config[i];

            robo::RobotConfig robot;
            try {
                robot.ns = robot_yaml["namespace"].as<std::string>();
            } catch (YAML::TypedBadConversion<std::string>& e) {
                echo::warn("Robot namespace not found in config file. Using default value.");
                robot.ns = "robot" + std::to_string(i);
            }

            try {
                robot.datum.altitude = datum[2].as<double>();
                robot.datum.latitude = datum[0].as<double>();
                robot.datum.longitude = datum[1].as<double>();
            } catch (YAML::TypedBadConversion<double>& e) {
                echo::error("Datum not found in config file. Exiting.");
            }

            try {
                const auto& initial_pose = robot_yaml["location"]["gnss"];
                auto x = initial_pose[0].as<double>();
                auto y = initial_pose[1].as<double>();
                auto z = 0;
                try {
                    z = initial_pose[2].as<double>();
                } catch (...) {
                    z = 0;
                }
                auto enu = gps_to_enu(robot.datum.latitude, robot.datum.longitude, robot.datum.altitude, x, y, z);
                robot.pose.x = std::get<0>(enu);
                robot.pose.y = std::get<1>(enu);
                robot.pose.z = std::get<2>(enu);
            } catch (...) {
                try {
                    const auto& initial_pose = robot_yaml["location"]["pose"];
                    robot.pose.x = initial_pose[0].as<double>();
                    robot.pose.y = initial_pose[1].as<double>();
                    try {
                        robot.pose.z = initial_pose[2].as<double>();
                    } catch (...) {
                        robot.pose.z = 0;
                    }
                } catch (...) {
                    echo::warn("Robot initial pose not found in config file. Using random GPS coordinates.");
                    auto pose = random_odom(20, 50);
                    robot.pose.x = std::get<0>(pose);
                    robot.pose.y = std::get<1>(pose);
                    robot.pose.z = std::get<2>(pose);
                    robot.pose.t = rand() % 360;
                }
            }

            try {
                robot.pose.t = robot_yaml["location"]["heading"].as<double>();
            } catch (...) {
                echo::warn("Robot initial heading not found in config file. Using random value.");
                robot.pose.t = rand() % 360;
            }

            try {
                robot.uuid = robot_yaml["info"]["uuid"].as<std::string>();
                robot.rci = robot_yaml["info"]["rci"].as<int8_t>();
            } catch (YAML::TypedBadConversion<std::string>& e) {
                echo::error("Robot info not found in config file. Exiting.");
            }

            // Sensors are hardcoded for now (customize as needed)
            robot.sensors = {true, true, true, true};

            // echo::info("Robot: {}", robo::to_string(robot));
            // Add the robot to the vector
            robots_.push_back(robot);
        }
    }

    std::vector<robo::RobotConfig> get_robots() {
        return robots_;
    }

    //TODO: GPS2ENU should be a universal function from localization package
    std::tuple<double, double, double> gps_to_enu(
        double datum_lat, double datum_lon, double datum_alt,
        double current_lat, double current_lon, double current_alt) {

        const double EARTH_RADIUS = 6378137.0;
        // Convert degrees to radians
        auto deg_to_rad = [](double deg) { return deg * M_PI / 180.0; };
        double datum_lat_rad = deg_to_rad(datum_lat);
        double datum_lon_rad = deg_to_rad(datum_lon);
        double current_lat_rad = deg_to_rad(current_lat);
        double current_lon_rad = deg_to_rad(current_lon);

        // Compute differences in latitude and longitude
        double dlat = current_lat_rad - datum_lat_rad;
        double dlon = current_lon_rad - datum_lon_rad;

        // Compute ENU coordinates
        double east = EARTH_RADIUS * dlon * cos(datum_lat_rad);
        double north = EARTH_RADIUS * dlat;
        double up = current_alt - datum_alt;

        return std::make_tuple(east, north, up);
    }

    std::tuple<double, double, double> random_odom(int min = 100, int max = 200) {
        if (min > max) {
            std::swap(min, max); // Ensure min is less than or equal to max
        }
        std::random_device rd_;
        std::mt19937 gen(rd_());

        // Generate a random magnitude
        std::uniform_int_distribution<int> dist_magnitude(min, max);

        // Generate a random sign (-1 or 1)
        std::uniform_int_distribution<int> dist_sign(0, 1);

        auto generate_value = [&]() -> double {
            int magnitude = dist_magnitude(gen);
            int sign = dist_sign(gen) == 0 ? -1 : 1; // Convert 0 to -1 for negative range
            return magnitude * sign;
        };

        auto x = generate_value();
        auto y = generate_value();
        auto z = 0.0;
        return std::make_tuple(x, y, z);
    }

};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto parser = ConfigParser("simulation.yaml");
    auto robots = parser.get_robots();

    rclcpp::NodeOptions options;
    options.parameter_overrides({{"use_sim_time", true}});
    auto node = rclcpp::Node::make_shared("farmbot_flatlands", options);


    auto simulator = std::make_shared<sim::SIM>(node, robots);
    simulator->create_world();
    simulator->start_simulation();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
