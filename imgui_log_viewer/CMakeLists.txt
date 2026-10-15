
cmake_minimum_required(VERSION 3.10)
project(imgui_log_viewer)

find_package(ament_cmake REQUIRED)
find_package(imgui_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()
find_package(Boost REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_library(imgui_log_viewer SHARED
    src/log_viewer.cpp
)
target_link_libraries(imgui_log_viewer PRIVATE
    imgui_ros::imgui_ros
    pluginlib::pluginlib
    Boost::headers
    ${rcl_interfaces_TARGETS}
)
target_link_options(imgui_log_viewer PRIVATE
    "-Wl,--no-undefined"
)

install(TARGETS imgui_log_viewer
    LIBRARY DESTINATION lib
)

pluginlib_export_plugin_description_file(imgui_ros imgui_plugin.xml)

ament_package()
