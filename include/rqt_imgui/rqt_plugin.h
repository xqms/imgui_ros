// rqt imgui integration
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef RQT_IMGUI_RQT_PLUGIN_H
#define RQT_IMGUI_RQT_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_2_Core>

#include "window.h"

struct ImGuiContext;
struct ImGuiIO;
struct ImPlotContext;

namespace rqt_imgui
{

class Widget : public QOpenGLWidget, protected QOpenGLFunctions_3_2_Core, public Context
{
public:
    class RQTSubscriber;

    explicit Widget(Window* window, const ros::NodeHandle& nh, QWidget* parent = nullptr);
    ~Widget();

    void shutdown();
    void registerSubscriber(RQTSubscriber* sub);
    void deregisterSubscriber(RQTSubscriber* sub);

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

    Subscriber subscribeRaw(const std::string& topic, int queue, const RawCb& rawCb, const ros::TransportHints& hints = {}) override;

    bool event(QEvent * event) override;


    Window* m_window = nullptr;

    ros::NodeHandle m_nh;

    ImGuiContext* m_imgui = {};
    ImGuiIO* m_io = {};
    ImPlotContext* m_implot = {};

    QTimer* m_updateTimer = nullptr;

    std::vector<RQTSubscriber*> m_subscribers;

    bool m_running = true;
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

private:
    WindowImpl m_window;
    Widget* m_w = nullptr;
};

}

#endif
