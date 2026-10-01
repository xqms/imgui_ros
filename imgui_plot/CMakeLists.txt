
cmake_minimum_required(VERSION 3.10)
project(imgui_plot)

find_package(ament_cmake REQUIRED)
find_package(imgui_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_library(imgui_plot SHARED
    src/gui.cpp
)
target_link_libraries(imgui_plot PRIVATE
    imgui_ros::imgui_ros
    pluginlib::pluginlib
    ${std_msgs_TARGETS}
)
target_link_options(imgui_plot PRIVATE
    "-Wl,--no-undefined"
)

install(TARGETS imgui_plot
    LIBRARY DESTINATION lib
)

pluginlib_export_plugin_description_file(imgui_ros imgui_plugin.xml)

ament_package()
