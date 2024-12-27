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
    // Set the use_sim_time parameter to true
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"use_sim_time", true}});
    // Create the SIM node
    auto simulator = std::make_shared<sim::SIM>(options);
    // Start the simulator (initialize plugins and start_simulation timer)
    simulator->create_world();
    simulator->start_simulation();
    // Create a multi-threaded executor with, for example, 4 threads
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    // Add the SIM node to the executor
    executor.add_node(simulator);
    // Spin the executor (runs callbacks in multiple threads)
    executor.spin();
    // Shutdown the executor
    rclcpp::shutdown();
    return 0;
}
