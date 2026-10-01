// Plugin provider for rqt
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <qt_gui_cpp/ros_pluginlib_plugin_provider.h>

#include <pluginlib/class_list_macros.hpp>

#include <imgui_ros/window.h>

#include <rclcpp/context.hpp>
#include <rclcpp/node.hpp>

#include <unistd.h>

#include "widget.h"

namespace rqt_imgui
{

class DirectPlugin : public qt_gui_cpp::Plugin
{
public:
    DirectPlugin(const std::shared_ptr<imgui_ros::Window>& window, const rclcpp::Node::SharedPtr& node)
     : m_window{window}
     , m_node{node}
    {}

    ~DirectPlugin() {}

    void initPlugin(qt_gui_cpp::PluginContext& ctx) override
    {
        m_w = new Widget(m_window, m_node);

        // Important, otherwise the window does not get deleted before the widget
        m_window.reset();
        m_node.reset();

        ctx.addWidget(m_w);
    }

    void shutdownPlugin() override
    {
        m_w->shutdown();
    }

    void saveSettings(qt_gui_cpp::Settings&, qt_gui_cpp::Settings& instanceSettings) const override
    { m_w->saveSettings(instanceSettings); }

    void restoreSettings(const qt_gui_cpp::Settings&, const qt_gui_cpp::Settings& instanceSettings) override
    { m_w->restoreSettings(instanceSettings); }

private:
    std::shared_ptr<imgui_ros::Window> m_window;
    rclcpp::Node::SharedPtr m_node;
    Widget* m_w = nullptr;
};

class ImguiROSPluginProvider : public qt_gui_cpp::RosPluginlibPluginProvider<imgui_ros::Window>
{
public:
    ImguiROSPluginProvider()
     : qt_gui_cpp::RosPluginlibPluginProvider<imgui_ros::Window>{"imgui_ros", "imgui_ros::Window"}
    {}

    ~ImguiROSPluginProvider()
    {
        if(m_context)
            m_context->shutdown("rqt_imgui plugin provider destroyed");
    }

    qt_gui_cpp::Plugin* load_plugin(const QString& plugin_id, qt_gui_cpp::PluginContext* plugin_context) override
    {
        if(!m_context)
        {
            // rqt_gui_cpp calls rclcpp::init() on the global context without checking whether it is
            // initialized already. Use our own context so that we do not get in each other's way.
            m_context = std::make_shared<rclcpp::Context>();
            m_context->init(0, nullptr);
        }

        std::string lookup_name = plugin_id.toStdString();

        std::shared_ptr<imgui_ros::Window> instance;
        try
        {
            instance = create_plugin(lookup_name, plugin_context);
        }
        catch(pluginlib::PluginlibException& e)
        {
            fprintf(stderr, "Could not load plugin %s: %s\n", lookup_name.c_str(), e.what());
            return {};
        }

        if(!instance)
        {
            fprintf(stderr, "pluginlib fail\n");
            return {};
        }

        // Each plugin gets its own node, which is spun from the widget's paint method.
        std::stringstream name;
        name << "rqt_imgui_" << getpid() << "_" << m_nodeCounter++;

        auto node = std::make_shared<rclcpp::Node>(name.str(), rclcpp::NodeOptions().context(m_context));

        auto plugin = new DirectPlugin(instance, node);

        if(plugin_context)
            plugin->initPlugin(*plugin_context);

        return plugin;
    }

    void unload(void* instance) override
    {
        delete reinterpret_cast<DirectPlugin*>(instance);
    }

private:
    rclcpp::Context::SharedPtr m_context;
    unsigned int m_nodeCounter = 0;
};

}

PLUGINLIB_EXPORT_CLASS(rqt_imgui::ImguiROSPluginProvider, qt_gui_cpp::PluginProvider)
