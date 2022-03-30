// OpenGL widget integration
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef RQT_IMGUI_WIDGET_H
#define RQT_IMGUI_WIDGET_H

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
    explicit Widget(const boost::shared_ptr<imgui_ros::Window>& window, const ros::NodeHandle& nh, QWidget* parent = nullptr);
    ~Widget();

    void saveSettings(qt_gui_cpp::Settings& settings);
    void restoreSettings(const qt_gui_cpp::Settings& settings);

    void shutdown();

private:

    void initializeGL() override;
    void resizeGL(int w, int h) override;

    void paintGL() override;

    void mouseMoveEvent(QMouseEvent * event) override;
    void mousePressEvent(QMouseEvent * event) override;
    void mouseReleaseEvent(QMouseEvent * event) override;

    void setWindowTitle(const std::string& windowTitle) override;
    void setUpdateRate(float rate) override;

    void updateCursor();

    ros::NodeHandle& nodeHandle() override
    { return m_nh; }

    boost::shared_ptr<imgui_ros::Window> m_window;

    std::unique_ptr<ros::CallbackQueue> m_callbackQueue;
    ros::NodeHandle m_nh;

    ImGuiContext* m_imgui = {};
    ImGuiIO* m_io = {};
    ImPlotContext* m_implot = {};

    QTimer* m_updateTimer = nullptr;

    bool m_running = true;

    std::optional<imgui_ros::Settings> m_queuedSettings;
};

}

#endif
