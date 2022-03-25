
cmake_minimum_required(VERSION 3.10)
project(rqt_imgui)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

find_package(PkgConfig)
pkg_check_modules(FontConfig fontconfig REQUIRED IMPORTED_TARGET)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rqt_gui_cpp
    imgui_ros
)

catkin_package(
    CATKIN_DEPENDS imgui_ros
    CFG_EXTRAS rqt_imgui.cmake
    LIBRARIES rqt_imgui
)

set(CMAKE_CXX_STANDARD 17)

add_library(rqt_imgui
    src/rqt_plugin.cpp
)
target_include_directories(rqt_imgui PRIVATE
    ${catkin_INCLUDE_DIRS}
    include
)
target_link_libraries(rqt_imgui
    ${catkin_LIBRARIES}
    Qt5::Widgets
    PkgConfig::FontConfig
)
target_link_options(rqt_imgui PRIVATE
    "-Wl,--no-undefined"
)

install(TARGETS rqt_imgui
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
    include/rqt_imgui/rqt_plugin.h
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
)
