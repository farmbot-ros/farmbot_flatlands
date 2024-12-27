#include "rclcpp/rclcpp.hpp"
#include "farmbot_flatlands/sim.hpp"
#include "farmbot_flatlands/robot.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
namespace aix = ament_index_cpp;
#include <spdlog/spdlog.h>
namespace echo = spdlog;

class ConfigParder {
    private:
        std::vector <robo::Robot> robots_;

    public:
        ConfigParder(std::string filename) {
            std::string pcg = aix::get_package_share_directory("farmbot_raph");
            std::string path = pcg + "/config/" + filename;
            echo::info("Reading config file: {}", path);
        }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.parameter_overrides({{"use_sim_time", true}});

    auto node = rclcpp::Node::make_shared("farmbot_flatlands", options);

    auto simulator = std::make_shared<sim::SIM>(node);
    simulator->create_world();
    simulator->start_simulation();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
