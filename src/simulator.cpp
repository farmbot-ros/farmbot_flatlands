#include "rclcpp/rclcpp.hpp"
#include "farmbot_flatlands/sim.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Set the use_sim_time parameter to true
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {{"use_sim_time", true}}
    );

    // Create the SIM node
    auto simulator = std::make_shared<sim::SIM>(options);

    // Start the simulator (initialize plugins and start timer)
    simulator->start();

    // Create a multi-threaded executor with, for example, 4 threads
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4 // Number of threads
    );

    // Add the SIM node to the executor
    executor.add_node(simulator);

    // Spin the executor (runs callbacks in multiple threads)
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
