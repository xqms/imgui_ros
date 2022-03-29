// rqt imgui integration
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef RQT_IMGUI_RQT_PLUGIN_H
#define RQT_IMGUI_RQT_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_2_Core>

#include <imgui_ros/window.h>

struct ImGuiContext;
struct ImGuiIO;
struct ImPlotContext;

namespace rqt_imgui
{

class Widget : public QOpenGLWidget, protected QOpenGLFunctions_3_2_Core, public imgui_ros::Context
{
public:
    class RQTSubscriber;

    explicit Widget(imgui_ros::Window* window, const ros::NodeHandle& nh, QWidget* parent = nullptr);
    ~Widget();

    void shutdown();
    void registerSubscriber(RQTSubscriber* sub);
    void deregisterSubscriber(RQTSubscriber* sub);

    void saveSettings(qt_gui_cpp::Settings& settings);
    void restoreSettings(const qt_gui_cpp::Settings& settings);

private:

    void initializeGL() override;
    void resizeGL(int w, int h) override;

    void paintGL() override;

    void mouseMoveEvent(QMouseEvent * event) override;
    void mousePressEvent(QMouseEvent * event) override;
    void mouseReleaseEvent(QMouseEvent * event) override;

    void setWindowTitle(const std::string& windowTitle) override;
    void setUpdateRate(float rate) override;

    ros::NodeHandle& nodeHandle() override
    { return m_nh; }

    imgui_ros::Subscriber subscribeRaw(const std::string& topic, int queue, const RawCb& rawCb, const ros::TransportHints& hints = {}) override;

    bool event(QEvent * event) override;


    imgui_ros::Window* m_window = nullptr;

    ros::NodeHandle m_nh;

    ImGuiContext* m_imgui = {};
    ImGuiIO* m_io = {};
    ImPlotContext* m_implot = {};

    QTimer* m_updateTimer = nullptr;

    std::vector<RQTSubscriber*> m_subscribers;

    bool m_running = true;

    std::optional<imgui_ros::Settings> m_queuedSettings;
};

template<class WindowImpl>
class RQTPlugin : public rqt_gui_cpp::Plugin
{
public:
    RQTPlugin() {}
    ~RQTPlugin() {}

    void initPlugin(qt_gui_cpp::PluginContext& ctx) override
    {
        m_w = new Widget(&m_window, getPrivateNodeHandle());
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
    WindowImpl m_window;
    Widget* m_w = nullptr;
};

}

#endif
