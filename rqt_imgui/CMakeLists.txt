cmake_minimum_required(VERSION 3.10)
project(rqt_imgui)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(imgui_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

find_package(PkgConfig)
pkg_check_modules(FontConfig fontconfig REQUIRED IMPORTED_TARGET)

set(CMAKE_CXX_STANDARD 17)

add_library(rqt_imgui_provider SHARED
    src/imgui_ros_plugin_provider.cpp
    src/widget.cpp
    src/nfd_support.cpp
)
target_link_libraries(rqt_imgui_provider PRIVATE
    imgui_ros::imgui_ros
    qt_gui_cpp::qt_gui_cpp
    pluginlib::pluginlib
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    Qt5::Widgets
    PkgConfig::FontConfig
)
target_link_options(rqt_imgui_provider PRIVATE
    "-Wl,--no-undefined"
)

install(TARGETS rqt_imgui_provider
    LIBRARY DESTINATION lib
)

pluginlib_export_plugin_description_file(qt_gui qt_gui_plugin.xml)

ament_package()
