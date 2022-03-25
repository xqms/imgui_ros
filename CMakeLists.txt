
cmake_minimum_required(VERSION 3.10)
project(rqt_imgui)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

find_package(PkgConfig)
pkg_check_modules(FontConfig fontconfig REQUIRED IMPORTED_TARGET)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rqt_gui_cpp
    topic_tools
)

catkin_package(
    CATKIN_DEPENDS rqt_gui_cpp
    CFG_EXTRAS rqt_imgui.cmake
    LIBRARIES rqt_imgui
)

set(CMAKE_CXX_STANDARD 17)

add_library(rqt_imgui_common
    # ImGui
    contrib/imgui/imgui.cpp
    contrib/imgui/imgui_demo.cpp
    contrib/imgui/imgui_draw.cpp
    contrib/imgui/imgui_tables.cpp
    contrib/imgui/imgui_widgets.cpp

    # ImPlot
    contrib/implot/implot.cpp
    contrib/implot/implot_items.cpp

    src/scrolling_buffer.cpp
)
target_include_directories(rqt_imgui_common PRIVATE
    include
)
target_include_directories(rqt_imgui_common PUBLIC
    contrib/imgui
    contrib/implot
)

add_library(rqt_imgui
    # ImGui
    contrib/imgui/backends/imgui_impl_opengl3.cpp

    src/rqt_plugin.cpp
)
target_include_directories(rqt_imgui PRIVATE
    ${catkin_INCLUDE_DIRS}
    include
)
target_link_libraries(rqt_imgui
    rqt_imgui_common
    ${catkin_LIBRARIES}
    Qt5::Widgets
    PkgConfig::FontConfig
)
target_link_options(rqt_imgui PRIVATE
    "-Wl,--no-undefined"
)

install(TARGETS rqt_imgui rqt_imgui_common
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
    include/rqt_imgui/rqt_plugin.h
    include/rqt_imgui/window.h
    include/rqt_imgui/scrolling_buffer.h
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
)

install(FILES
    contrib/imgui/imgui.h
    contrib/imgui/imconfig.h
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/imgui/
)

install(FILES
    contrib/implot/implot.h
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/implot/
)
