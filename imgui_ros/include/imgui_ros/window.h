// imgui ROS integration
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef IMGUI_ROS_WINDOW_H
#define IMGUI_ROS_WINDOW_H

#include "context.h"

#include <string>

namespace imgui_ros
{

class Window
{
public:
    virtual ~Window();

    virtual void initialize(Context* context) = 0;
    virtual void resize(int w, int h) {};
    virtual void paint() = 0;
};

}

#endif
