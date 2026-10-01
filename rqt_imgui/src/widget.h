// OpenGL widget integration
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef RQT_IMGUI_WIDGET_H
#define RQT_IMGUI_WIDGET_H

#include <qt_gui_cpp/settings.h>

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_2_Core>

#include <imgui_ros/window.h>

#include <rclcpp/executors/single_threaded_executor.hpp>

#include <fontconfig/fontconfig.h>

struct ImGuiContext;
struct ImGuiIO;
struct ImPlotContext;

namespace rqt_imgui
{

class Widget : public QOpenGLWidget, protected QOpenGLFunctions_3_2_Core, public imgui_ros::Context
{
public:
    explicit Widget(const std::shared_ptr<imgui_ros::Window>& window, const rclcpp::Node::SharedPtr& node, QWidget* parent = nullptr);
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
    ImFont* loadFont(const std::string& query, float relativeSize = 1.0f) override;

    void setupNFDHandle(nfdwindowhandle_t* handle) override;

    void updateCursor();

    const rclcpp::Node::SharedPtr& node() override
    { return m_node; }

    std::shared_ptr<imgui_ros::Window> m_window;

    rclcpp::Node::SharedPtr m_node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;

    ImGuiContext* m_imgui = {};
    ImGuiIO* m_io = {};
    ImPlotContext* m_implot = {};

    FcConfig* m_fontConfig = {};

    QTimer* m_updateTimer = nullptr;

    bool m_running = true;

    std::optional<imgui_ros::Settings> m_queuedSettings;

    int m_defaultFontSize = -1;
};

}

#endif
