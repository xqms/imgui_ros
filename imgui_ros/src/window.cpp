// imgui ROS integration
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <imgui_ros/window.h>

namespace imgui_ros
{

Window::~Window()
{
}

void Window::setState(const std::map<std::string, std::string>&)
{
}

std::map<std::string, std::string> Window::getState() const
{
    return {};
}

}
