
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

catkin_package()

set(CMAKE_CXX_STANDARD 17)

add_library(rqt_imgui_provider
    src/imgui_ros_plugin_provider.cpp
    src/widget.cpp
    src/nfd_support.cpp
)
target_include_directories(rqt_imgui_provider PRIVATE
    ${catkin_INCLUDE_DIRS}
    include
)
target_link_libraries(rqt_imgui_provider
    ${catkin_LIBRARIES}
    Qt5::Widgets
    PkgConfig::FontConfig
)
target_link_options(rqt_imgui_provider PRIVATE
    "-Wl,--no-undefined"
)

install(TARGETS rqt_imgui_provider
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
    qt_gui_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
