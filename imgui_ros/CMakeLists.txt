
cmake_minimum_required(VERSION 3.10)
project(imgui_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()
find_package(Boost COMPONENTS program_options REQUIRED)

include(cmake/CPM.cmake)

CPMAddPackage(
    NAME glfw
    VERSION 3.4
    URL "https://github.com/glfw/glfw/releases/download/3.4/glfw-3.4.zip"
    URL_HASH "SHA256=b5ec004b2712fd08e8861dc271428f048775200a2df719ccf575143ba749a3e9"
    PATCHES contrib/glfw-wayland-fbsize.patch
    OPTIONS
      "GLFW_LIBRARY_TYPE STATIC"
      "GLFW_INSTALL OFF"
)

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FontConfig fontconfig IMPORTED_TARGET)
pkg_check_modules(freetype freetype2 IMPORTED_TARGET)
pkg_check_modules(DBus dbus-1 IMPORTED_TARGET)

set(CMAKE_CXX_STANDARD 20)

# The COMMANDs of one execute_process() run concurrently as a pipeline, so the
# directory has to exist before it starts.
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/include/imgui_ros")
execute_process(
    COMMAND_ERROR_IS_FATAL ANY
    COMMAND cmake -E create_symlink "${CMAKE_CURRENT_SOURCE_DIR}/contrib/imgui" "${CMAKE_CURRENT_BINARY_DIR}/include/imgui_ros/imgui"
    COMMAND cmake -E create_symlink "${CMAKE_CURRENT_SOURCE_DIR}/contrib/implot" "${CMAKE_CURRENT_BINARY_DIR}/include/imgui_ros/implot"
    COMMAND cmake -E create_symlink "${CMAKE_CURRENT_SOURCE_DIR}/contrib/nativefiledialog-extended/src/include" "${CMAKE_CURRENT_BINARY_DIR}/include/imgui_ros/nfd"
)

add_library(imgui_ros SHARED
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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/contrib/imgui>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}/imgui_ros/imgui>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/contrib/implot>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}/imgui_ros/implot>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/contrib/nativefiledialog-extended/src/include>
)
target_link_libraries(imgui_ros PUBLIC
    dl
    rclcpp::rclcpp
)
target_link_libraries(imgui_ros PRIVATE
    PkgConfig::DBus
    PkgConfig::freetype
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
target_link_libraries(gui
    imgui_ros
    glfw
    OpenGL::OpenGL
    PkgConfig::FontConfig
    rclcpp::rclcpp
    pluginlib::pluginlib
    ament_index_cpp::ament_index_cpp
    Boost::program_options
)


install(TARGETS gui
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

install(
    TARGETS imgui_ros
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

install(FILES
    contrib/imgui/imgui.h
    contrib/imgui/imgui_internal.h
    contrib/imgui/imstb_textedit.h
    contrib/imgui/imconfig.h
    DESTINATION include/${PROJECT_NAME}/imgui_ros/imgui/
)

install(FILES
    contrib/imgui/backends/imgui_impl_opengl3.h
    DESTINATION include/${PROJECT_NAME}/imgui_ros/imgui/backends/
)

install(FILES
    contrib/implot/implot.h
    DESTINATION include/${PROJECT_NAME}/imgui_ros/implot/
)

install(FILES
    contrib/nativefiledialog-extended/src/include/nfd.h
    DESTINATION include/${PROJECT_NAME}/imgui_ros/nfd/
)

install(FILES
    contrib/fonts/fa-solid-900.ttf
    DESTINATION share/${PROJECT_NAME}/contrib/fonts/
)

ament_package()

