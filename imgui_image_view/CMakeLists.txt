
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

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL)

find_package(X11)

add_library(imgui_image_view
    src/image_view.cpp
    src/decoder.cpp
)
target_link_libraries(imgui_image_view
    ${catkin_LIBRARIES}
    PkgConfig::avcodec
    PkgConfig::avformat
    PkgConfig::avutil
    PkgConfig::swscale
    OpenGL::GL
    OpenGL::GLX
    ${X11_LIBRARIES}
)
target_link_options(imgui_image_view PRIVATE
    "-Wl,--no-undefined"
)

install(TARGETS imgui_image_view
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES imgui_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
