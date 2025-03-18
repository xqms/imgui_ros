
cmake_minimum_required(VERSION 3.10)
project(imgui_ros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pluginlib
)

catkin_package(
    CATKIN_DEPENDS pluginlib roscpp
    CFG_EXTRAS imgui_ros.cmake
    LIBRARIES imgui_ros
)

find_package(glfw3 REQUIRED)
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FontConfig fontconfig IMPORTED_TARGET)
pkg_check_modules(freetype freetype2 IMPORTED_TARGET)
pkg_check_modules(DBus dbus-1 IMPORTED_TARGET)

set(CMAKE_CXX_STANDARD 17)

execute_process(
    COMMAND cmake -E make_directory "${CATKIN_DEVEL_PREFIX}/include/imgui_ros"
    COMMAND cmake -E create_symlink "${CMAKE_CURRENT_SOURCE_DIR}/contrib/imgui" "${CATKIN_DEVEL_PREFIX}/include/imgui_ros/imgui"
    COMMAND cmake -E create_symlink "${CMAKE_CURRENT_SOURCE_DIR}/contrib/implot" "${CATKIN_DEVEL_PREFIX}/include/imgui_ros/implot"
    COMMAND cmake -E create_symlink "${CMAKE_CURRENT_SOURCE_DIR}/contrib/nativefiledialog-extended/src/include" "${CATKIN_DEVEL_PREFIX}/include/imgui_ros/nfd"
)

add_library(imgui_ros
    # ImGui
    contrib/imgui/imgui.cpp
    contrib/imgui/imgui_demo.cpp
    contrib/imgui/imgui_draw.cpp
    contrib/imgui/imgui_tables.cpp
    contrib/imgui/imgui_widgets.cpp
    contrib/imgui/backends/imgui_impl_opengl3.cpp
    contrib/imgui/misc/freetype/imgui_freetype.cpp

    # ImPlot
    contrib/implot/implot.cpp
    contrib/implot/implot_items.cpp

    # NFD
    contrib/nativefiledialog-extended/src/nfd_portal.cpp

    src/context.cpp
    src/scrolling_buffer.cpp
    src/topic_selector.cpp
    src/window.cpp
)
target_compile_definitions(imgui_ros PUBLIC
    IMGUI_ENABLE_FREETYPE=1
)
target_include_directories(imgui_ros PUBLIC
    include
    contrib/imgui
    contrib/implot
    contrib/nativefiledialog-extended/src/include
    ${CATKIN_DEVEL_PREFIX}/include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(imgui_ros PUBLIC
    dl
    ${roscpp_LIBRARIES}
    PkgConfig::freetype
)
target_link_libraries(imgui_ros PRIVATE
    PkgConfig::DBus
)
target_link_options(imgui_ros PRIVATE
    "-Wl,--no-undefined"
)


add_executable(gui
    src/gui.cpp
    src/nfd_support.cpp
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
    PkgConfig::FontConfig
    ${catkin_LIBRARIES}
)


install(TARGETS imgui_ros gui
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
    include/imgui_ros/box.h
    include/imgui_ros/context.h
    include/imgui_ros/icons.h
    include/imgui_ros/math.h
    include/imgui_ros/scrolling_buffer.h
    include/imgui_ros/topic_selector.h
    include/imgui_ros/window.h
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
)

install(FILES
    contrib/imgui/imgui.h
    contrib/imgui/imgui_internal.h
    contrib/imgui/imstb_textedit.h
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

install(FILES
    contrib/nativefiledialog-extended/src/include/nfd.h
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/nfd/
)

install(FILES
    contrib/fonts/fa-solid-900.ttf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/contrib/fonts/
)
