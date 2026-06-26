
cmake_minimum_required(VERSION 3.10)
project(imgui_image_view)

find_package(ament_cmake REQUIRED)
find_package(imgui_ros REQUIRED)
find_package(rosfmt REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(fmt REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_STANDARD 20)

find_package(PkgConfig)
pkg_check_modules(avcodec IMPORTED_TARGET libavcodec)
pkg_check_modules(avformat IMPORTED_TARGET libavformat)
pkg_check_modules(avutil IMPORTED_TARGET libavutil)
pkg_check_modules(swscale IMPORTED_TARGET libswscale)
pkg_check_modules(libpng IMPORTED_TARGET libpng)

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL)

find_package(X11)

find_package(CUDAToolkit)
if(CUDAToolkit_FOUND)
    message(STATUS "CUDA found, enabling CUDA decoder")
    add_definitions(-DCUDA_DECODER=1)
endif()

add_library(imgui_image_view SHARED
    src/image_view.cpp
    src/decoder.cpp
    src/cuda_decoder.cpp
)
target_link_libraries(imgui_image_view PRIVATE
    imgui_ros::imgui_ros
    rosfmt::rosfmt
    fmt::fmt
    pluginlib::pluginlib
    ${sensor_msgs_TARGETS}
    PkgConfig::avcodec
    PkgConfig::avformat
    PkgConfig::avutil
    PkgConfig::swscale
    PkgConfig::libpng
    OpenGL::GL
    OpenGL::GLX
    OpenGL::EGL
    ${X11_LIBRARIES}
)
target_link_options(imgui_image_view PRIVATE
    "-Wl,--no-undefined"
)

if(CUDAToolkit_FOUND)
    target_link_libraries(imgui_image_view PRIVATE
        CUDA::cudart
        CUDA::cuda_driver
        CUDA::nppicc
        CUDA::nppidei
    )
endif()

install(TARGETS imgui_image_view
    LIBRARY DESTINATION lib
)

pluginlib_export_plugin_description_file(imgui_ros imgui_plugin.xml)

ament_package()

