// imgui ROS integration
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <imgui_ros/window.h>

namespace imgui_ros
{

Window::~Window()
{
}

void Window::setContext(Context* ctx)
{
    m_context = ctx;
}

void Window::setState(const Settings&)
{
}

Settings Window::getState() const
{
    return {};
}

void Window::resize(int, int)
{
}

}
