
cmake_minimum_required(VERSION 3.10)
project(imgui_ros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    topic_tools
)

catkin_package(
    CATKIN_DEPENDS topic_tools
    CFG_EXTRAS imgui_ros.cmake
    LIBRARIES imgui_ros
)

set(CMAKE_CXX_STANDARD 17)

add_library(imgui_ros
    # ImGui
    contrib/imgui/imgui.cpp
    contrib/imgui/imgui_demo.cpp
    contrib/imgui/imgui_draw.cpp
    contrib/imgui/imgui_tables.cpp
    contrib/imgui/imgui_widgets.cpp
    contrib/imgui/backends/imgui_impl_opengl3.cpp

    # ImPlot
    contrib/implot/implot.cpp
    contrib/implot/implot_items.cpp

    src/scrolling_buffer.cpp
)
target_include_directories(imgui_ros PRIVATE
    include
)
target_include_directories(imgui_ros PUBLIC
    contrib/imgui
    contrib/implot
)
target_link_libraries(imgui_ros PRIVATE
    dl
)
target_link_options(imgui_ros PRIVATE
    "-Wl,--no-undefined"
)

install(TARGETS imgui_ros
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
    include/imgui_ros/window.h
    include/imgui_ros/scrolling_buffer.h
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
)

install(FILES
    contrib/imgui/imgui.h
    contrib/imgui/imconfig.h
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/imgui/
)

install(FILES
    contrib/imgui/backends/imgui_impl_opengl3.h
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/imgui/backends/
)

install(FILES
    contrib/implot/implot.h
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/implot/
)
