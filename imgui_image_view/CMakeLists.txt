
cmake_minimum_required(VERSION 3.10)
project(imgui_image_view)

find_package(catkin REQUIRED COMPONENTS
    imgui_ros
    rosfmt
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_STANDARD 17)

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

add_library(imgui_image_view
    src/image_view.cpp
    src/decoder.cpp
    src/cuda_decoder.cpp
)
target_link_libraries(imgui_image_view
    ${catkin_LIBRARIES}
    PkgConfig::avcodec
    PkgConfig::avformat
    PkgConfig::avutil
    PkgConfig::swscale
    PkgConfig::libpng
    OpenGL::GL
    OpenGL::GLX
    ${X11_LIBRARIES}
)
target_link_options(imgui_image_view PRIVATE
    "-Wl,--no-undefined"
)

if(CUDAToolkit_FOUND)
    target_link_libraries(imgui_image_view
        CUDA::cudart
        CUDA::cuda_driver
        CUDA::nppicc
        CUDA::nppidei
    )
endif()

install(TARGETS imgui_image_view
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES imgui_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
