cmake_minimum_required(VERSION 3.15)
project(farmbot_flatlands)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()
cmake_policy(SET CMP0074 NEW)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(FetchContent)

fetchcontent_declare(
    muli
    GIT_REPOSITORY https://github.com/bresilla/Muli
)
FetchContent_MakeAvailable(muli)
include_directories(${muli_SOURCE_DIR}/include)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)


set(executables
    visualize
    simulator
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(farmbot_interfaces REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rviz_common_INCLUDE_DIRS}
    ${rviz_rendering_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${nlohmann_json_INCLUDE_DIRS}
)

set(dependencies_ros
    rclcpp
    std_msgs
    std_srvs
    nav_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    tf2
    tf2_ros
    message_filters
    pcl_ros
    pcl_conversions
    farmbot_interfaces
    rviz_common
    rviz_rendering
    pluginlib
    diagnostic_msgs
    diagnostic_updater
)


find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
link_directories(${Eigen_LIBRARY_DIRS})
add_definitions(${Eigen_DEFINITIONS})
find_package(fmt REQUIRED)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(nlohmann_json REQUIRED)
#find curl
find_package(CURL REQUIRED)
find_package(Qt5Widgets REQUIRED)


set(dependencies_sys
    muli
    fmt::fmt
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Eigen_LIBRARIES}
    nlohmann_json::nlohmann_json
    ${CURL_LIBRARIES}
    Qt5::Widgets
)

foreach(executable ${executables})
    add_executable(${executable} src/${executable}.cpp)
    ament_target_dependencies(${executable} ${dependencies_ros})
    target_link_libraries(${executable} ${dependencies_sys})
endforeach()

install(TARGETS
    ${executables}
    DESTINATION lib/${PROJECT_NAME}
)


# add_library(polygon_tool_plugin SHARED rviz/polygon_tool.cc rviz/polygon_tool.h)
# ament_target_dependencies(polygon_tool_plugin ${dependencies_ros})
# target_link_libraries(polygon_tool_plugin ${dependencies_sys})

# install(TARGETS polygon_tool_plugin
#   DESTINATION lib/polygon_tool_plugin
# )

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# # Install the library
# install(TARGETS
#     rviz_polygon_tool_plugin EXPORT
#     ${PROJECT_NAME}-targets DESTINATION lib)
# ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
# pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()
