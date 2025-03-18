// Support methods for NFD
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NFD_SUPPORT_H
#define NFD_SUPPORT_H

#include <imgui_ros/nfd/nfd.h>

#include <GLFW/glfw3.h>

namespace nfd_support
{
    void setupWindowHandle(GLFWwindow* window, nfdwindowhandle_t* handle);
}

#endif
