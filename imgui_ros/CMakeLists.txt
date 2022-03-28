
cmake_minimum_required(VERSION 3.10)
project(imgui_ros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    topic_tools
    pluginlib
)

catkin_package(
    CATKIN_DEPENDS topic_tools
    CFG_EXTRAS imgui_ros.cmake
    LIBRARIES imgui_ros
)

find_package(glfw3)
find_package(OpenGL REQUIRED)

set(CMAKE_CXX_STANDARD 17)

execute_process(
    COMMAND cmake -E make_directory "${CATKIN_DEVEL_PREFIX}/include/imgui_ros"
    COMMAND cmake -E create_symlink "${CMAKE_CURRENT_SOURCE_DIR}/contrib/imgui" "${CATKIN_DEVEL_PREFIX}/include/imgui_ros/imgui"
    COMMAND cmake -E create_symlink "${CMAKE_CURRENT_SOURCE_DIR}/contrib/implot" "${CATKIN_DEVEL_PREFIX}/include/imgui_ros/implot"
)

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
    src/context.cpp
    src/window.cpp
)
target_include_directories(imgui_ros PUBLIC
    include
    contrib/imgui
    contrib/implot
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(imgui_ros PRIVATE
    dl
)
target_link_options(imgui_ros PRIVATE
    "-Wl,--no-undefined"
)


add_executable(gui
    src/gui.cpp
    contrib/imgui/backends/imgui_impl_opengl3.cpp
    contrib/imgui/backends/imgui_impl_glfw.cpp
)
target_include_directories(gui PRIVATE
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(gui
    imgui_ros
    glfw
    OpenGL::OpenGL
    ${catkin_LIBRARIES}
)


install(TARGETS imgui_ros gui
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
    include/imgui_ros/context.h
    include/imgui_ros/scrolling_buffer.h
    include/imgui_ros/window.h
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
