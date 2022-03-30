// rqt imgui integration
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "widget.h"

#include <QMouseEvent>
#include <QTimer>
#include <QCoreApplication>

#include <imgui_ros/imgui/imgui.h>
#include <imgui_ros/implot/implot.h>

#include <fontconfig/fontconfig.h>

#include <imgui_ros/imgui/backends/imgui_impl_opengl3.h>

#include <ros/callback_queue.h>

using namespace imgui_ros;

namespace
{
    ImGuiMouseButton imGuiButton(Qt::MouseButton btn)
    {
        switch(btn)
        {
            case Qt::LeftButton: return ImGuiMouseButton_Left;
            case Qt::MiddleButton: return ImGuiMouseButton_Middle;
            case Qt::RightButton: return ImGuiMouseButton_Right;
            default: return -1;
        }
    }
}

namespace rqt_imgui
{

Widget::Widget(const boost::shared_ptr<Window>& window, const ros::NodeHandle& nh, QWidget* parent)
 : QOpenGLWidget{parent}
 , m_window{window}
 , m_nh{nh}
{
    m_callbackQueue = std::make_unique<ros::CallbackQueue>();
    m_nh.setCallbackQueue(m_callbackQueue.get());

    m_updateTimer = new QTimer(this);
    connect(m_updateTimer, &QTimer::timeout, this, [&](){ update(); });
    m_updateTimer->start(50);

    auto fmt = QSurfaceFormat::defaultFormat();
    fmt.setSamples(4);
    setFormat(fmt);

    setMouseTracking(true);
}

Widget::~Widget()
{
    if(m_implot)
        ImPlot::DestroyContext(m_implot);

    if(m_imgui)
        ImGui::DestroyContext(m_imgui);
}

void Widget::setUpdateRate(float updateRate)
{
    int interval = 1000.0 / updateRate;
    if(m_updateTimer->interval() != interval)
        m_updateTimer->setInterval(interval);
}

void Widget::mouseMoveEvent(QMouseEvent* event)
{
    if(!m_imgui)
        return;

    ImGui::SetCurrentContext(m_imgui);
    m_io->AddMousePosEvent(event->x(), event->y());
}

void Widget::mousePressEvent(QMouseEvent* event)
{
    if(!m_imgui)
        return;

    int button = imGuiButton(event->button());
    if(button < 0)
        return;

    ImGui::SetCurrentContext(m_imgui);
    m_io->AddMouseButtonEvent(button, true);
}

void Widget::mouseReleaseEvent(QMouseEvent* event)
{
    if(!m_imgui)
        return;

    int button = imGuiButton(event->button());
    if(button < 0)
        return;

    ImGui::SetCurrentContext(m_imgui);
    m_io->AddMouseButtonEvent(button, false);
}

void Widget::initializeGL()
{
    initializeOpenGLFunctions();

    IMGUI_CHECKVERSION();
    m_imgui = ImGui::CreateContext();
    ImGui::SetCurrentContext(m_imgui);

    m_io = &ImGui::GetIO();
    m_io->IniFilename = nullptr;
    ImGui::StyleColorsLight(&ImGui::GetStyle());
    ImGui::GetStyle().Colors[ImGuiCol_WindowBg] = {1.0f, 1.0f, 1.0f, 1.0f};

    m_implot = ImPlot::CreateContext();
    ImPlot::SetCurrentContext(m_implot);
    ImPlot::StyleColorsLight();
    ImPlot::GetStyle().Colors[ImPlotCol_AxisBgHovered] = ImPlot::GetStyle().Colors[ImPlotCol_AxisBg];

    ImGui_ImplOpenGL3_Init("#version 130");

    std::string fontFile;
    {
        FcConfig* config = FcInitLoadConfigAndFonts();

        // configure the search pattern,
        // assume "name" is a std::string with the desired font name in it
        auto fontName = fontInfo().family().toLocal8Bit();
        FcPattern* pat = FcNameParse((const FcChar8*)(fontName.data()));
        FcConfigSubstitute(config, pat, FcMatchPattern);
        FcDefaultSubstitute(pat);

        // find the font
        FcResult res;
        FcPattern* font = FcFontMatch(config, pat, &res);
        if (font)
        {
            FcChar8* file = NULL;
            if (FcPatternGetString(font, FC_FILE, 0, &file) == FcResultMatch)
            {
                // save the file to another std::string
                fontFile = (char*)file;
            }
            FcPatternDestroy(font);
        }

        FcPatternDestroy(pat);
    }

    if(!fontFile.empty())
    {
        m_io->Fonts->AddFontFromFileTTF(fontFile.c_str(), fontInfo().pixelSize(), NULL, NULL);
    }

    m_window->setContext(this);
    m_window->initialize();

    if(m_queuedSettings)
        m_window->setState(*m_queuedSettings);
}

void Widget::resizeGL(int w, int h)
{
    m_io->DisplaySize.x = w;
    m_io->DisplaySize.y = h;

    m_window->resize(w, h);
}

void Widget::paintGL()
{
    if(!m_window)
        return;

    ImGui::SetCurrentContext(m_imgui);
    ImPlot::SetCurrentContext(m_implot);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos({0,0});
    ImGui::SetNextWindowSize(m_io->DisplaySize);
    ImGui::Begin("window", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoResize);

    // Invoke subscriber/box callbacks
    m_callbackQueue->callAvailable();

    m_window->paint();

    ImGui::End();

    ImGui::Render();
    glViewport(0, 0, width(), height());
    glClearColor(1.0, 1.0, 1.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void Widget::setWindowTitle(const std::string& windowTitle)
{
    QWidget::setWindowTitle(QString::fromStdString(windowTitle));
}

void Widget::saveSettings(qt_gui_cpp::Settings& settings)
{
    auto values = m_window->getState();

    for(auto& pair : values)
        settings.setValue(QString::fromStdString(pair.first), QString::fromStdString(pair.second));
}

void Widget::restoreSettings(const qt_gui_cpp::Settings& settings)
{
    imgui_ros::Settings values;
    for(auto key : settings.childKeys())
        values[key.toStdString()] = settings.value(key).toString().toStdString();

    // If the context has not been initialized yet, delay settings restoration
    if(!m_imgui)
    {
        m_queuedSettings = values;
        return;
    }

    m_window->setState(values);
}

void Widget::shutdown()
{
    m_window.reset();
}

}
