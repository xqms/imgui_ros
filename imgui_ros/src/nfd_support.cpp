// Support methods for NFD
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "nfd_support.h"

#define GLFW_EXPOSE_NATIVE_X11
#include <imgui_ros/nfd/nfd_glfw3.h>

#include <cstdio>

namespace nfd_support
{
    void setupWindowHandle(GLFWwindow* window, nfdwindowhandle_t* handle)
    {
        if(!NFD_GetNativeWindowFromGLFWWindow(window, handle))
            fprintf(stderr, "Could not get NFD window handle from GLFW\n");
    }
}
