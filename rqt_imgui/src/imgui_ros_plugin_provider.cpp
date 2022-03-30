// Plugin provider for rqt
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <qt_gui_cpp/ros_pluginlib_plugin_provider.h>

#include <pluginlib/class_list_macros.hpp>

#include <imgui_ros/window.h>

#include <QMessageBox>
#include <QMetaObject>

#include <thread>

#include "widget.h"

namespace rqt_imgui
{

class DirectPlugin : public qt_gui_cpp::Plugin
{
public:
    DirectPlugin(const boost::shared_ptr<imgui_ros::Window>& window, const ros::NodeHandle& nh)
     : m_window{window}
     , m_nh{nh}
    {}

    ~DirectPlugin() {}

    void initPlugin(qt_gui_cpp::PluginContext& ctx) override
    {
        m_w = new Widget(m_window, m_nh);

        // Important, otherwise the window does not get deleted before the widget
        m_window.reset();

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
    boost::shared_ptr<imgui_ros::Window> m_window;
    ros::NodeHandle m_nh;
    Widget* m_w = nullptr;
};

class ImguiROSPluginProvider : public qt_gui_cpp::RosPluginlibPluginProvider<imgui_ros::Window>
{
public:
    ImguiROSPluginProvider()
     : qt_gui_cpp::RosPluginlibPluginProvider<imgui_ros::Window>{"imgui_ros", "imgui_ros::Window"}
    {}

    ~ImguiROSPluginProvider()
    {}

    qt_gui_cpp::Plugin* load_plugin(const QString& plugin_id, qt_gui_cpp::PluginContext* plugin_context) override
    {
        if(!m_nh)
        {
            int argc = 0;
            char** argv = 0;
            std::stringstream name;
            name << "rqt_gui_cpp_node_";
            name << getpid();

            ros::init(argc, argv, name.str(), ros::init_options::NoSigintHandler);

            if(!ros::master::check())
            {
                auto dlg = new QMessageBox(
                    QMessageBox::Question,
                    QObject::tr("Waiting for ROS master"),
                    QObject::tr("Could not find ROS master. Either start a 'roscore' or abort loading the plugin."),
                    QMessageBox::Abort
                );

                std::atomic_bool abort = false;
                std::thread thread{[&](){
                    while(!abort)
                    {
                        if(ros::master::check())
                        {
                            QMetaObject::invokeMethod(dlg, "done", Qt::QueuedConnection, Q_ARG(int, QMessageBox::Ok));
                            return;
                        }

                        usleep(500*1000);
                    }
                }};

                int btn = dlg->exec();

                if(btn != QMessageBox::Ok)
                    abort = true;

                thread.join();

                if(btn != QMessageBox::Ok)
                    return nullptr;
            }

            m_nh = std::make_unique<ros::NodeHandle>("~");
        }

        std::string lookup_name = plugin_id.toStdString();

        auto instance = create_plugin(lookup_name, plugin_context);

        if(!instance)
        {
            fprintf(stderr, "pluginlib fail\n");
            return {};
        }

        auto plugin = new DirectPlugin(instance, *m_nh);

        if(plugin_context)
            plugin->initPlugin(*plugin_context);

        return plugin;
    }

    void unload(void* instance) override
    {
        delete reinterpret_cast<DirectPlugin*>(instance);
    }

private:
    std::unique_ptr<ros::NodeHandle> m_nh;
};

}

PLUGINLIB_EXPORT_CLASS(rqt_imgui::ImguiROSPluginProvider, qt_gui_cpp::PluginProvider)
