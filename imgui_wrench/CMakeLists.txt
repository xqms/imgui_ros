
cmake_minimum_required(VERSION 3.10)
project(imgui_wrench)

find_package(catkin REQUIRED COMPONENTS
    imgui_ros
    rosgraph_msgs
)

catkin_package()

set(CMAKE_CXX_STANDARD 17)

include_directories(${catkin_INCLUDE_DIRS})

add_library(imgui_wrench
    src/gui.cpp
)
target_link_libraries(imgui_wrench
    ${catkin_LIBRARIES}
)
target_link_options(imgui_wrench PRIVATE
    "-Wl,--no-undefined"
)

install(TARGETS imgui_wrench
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES imgui_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
