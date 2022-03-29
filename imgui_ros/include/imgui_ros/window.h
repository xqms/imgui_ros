// imgui ROS integration
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef IMGUI_ROS_WINDOW_H
#define IMGUI_ROS_WINDOW_H

#include "context.h"

#include <string>

namespace imgui_ros
{

class Settings : public std::map<std::string, std::string>
{
public:
    using std::map<std::string, std::string>::map;

    std::optional<std::string> get(const std::string& key) const
    {
        auto it = find(key);
        if(it != end())
            return {it->second};
        else
            return {};
    }
};

class Window
{
public:
    virtual ~Window();

    virtual void initialize(Context* context) = 0;
    virtual void resize(int w, int h) {};
    virtual void paint() = 0;

    virtual void setState(const Settings& settings);
    virtual Settings getState() const;
};

}

#endif
