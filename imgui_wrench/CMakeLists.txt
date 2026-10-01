
cmake_minimum_required(VERSION 3.10)
project(imgui_wrench)

find_package(ament_cmake REQUIRED)
find_package(imgui_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_library(imgui_wrench SHARED
    src/gui.cpp
)
target_link_libraries(imgui_wrench PRIVATE
    imgui_ros::imgui_ros
    pluginlib::pluginlib
    ${geometry_msgs_TARGETS}
)
target_link_options(imgui_wrench PRIVATE
    "-Wl,--no-undefined"
)

install(TARGETS imgui_wrench
    LIBRARY DESTINATION lib
)

pluginlib_export_plugin_description_file(imgui_ros imgui_plugin.xml)

ament_package()
