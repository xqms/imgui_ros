// rqt imgui integration
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <rqt_imgui/rqt_plugin.h>

#include <QMouseEvent>
#include <QTimer>
#include <QCoreApplication>

#include <imgui_ros/imgui/imgui.h>
#include <imgui_ros/imgui/implot.h>

#include <fontconfig/fontconfig.h>

#include <imgui_ros/imgui/backends/imgui_impl_opengl3.h>

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

    class MessageEvent : public QEvent
    {
    public:
        static const QEvent::Type Type;

        using Msg = boost::shared_ptr<const topic_tools::ShapeShifter>;
        using Cb = std::function<void(const Msg&)>;

        MessageEvent(const Msg& msg, const Cb& cb)
         : QEvent(Type)
         , m_msg{msg}
         , m_cb{cb}
        {}

        void call()
        {
            m_cb(m_msg);
        }

    private:
        Msg m_msg;
        Cb m_cb;
    };

    const QEvent::Type MessageEvent::Type = static_cast<QEvent::Type>(QEvent::registerEventType());
}

namespace rqt_imgui
{

class Widget::RQTSubscriber : public ros_imgui::Subscriber::Impl
{
public:
    RQTSubscriber(ros::Subscriber&& sub, rqt_imgui::Widget* w)
     : m_sub{std::move(sub)}
     , m_w{w}
    {
        w->registerSubscriber(this);
    }

    virtual ~RQTSubscriber()
    {
        m_w->deregisterSubscriber(this);
    }

    int getNumPublishers() override
    {
        return m_sub.getNumPublishers();
    }

    void shutdown() override
    {
        m_sub.shutdown();
    }

    void release()
    {
        m_w = nullptr;
    }

private:
    ros::Subscriber m_sub;
    rqt_imgui::Widget* m_w = nullptr;
};

Widget::Widget(Window* window, const ros::NodeHandle& nh, QWidget* parent)
 : QOpenGLWidget{parent}
 , m_window{window}
 , m_nh{nh}
{
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

    for(auto& sub : m_subscribers)
        sub->release();
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

    m_window->initialize(this);
}

void Widget::resizeGL(int w, int h)
{
    m_io->DisplaySize.x = w;
    m_io->DisplaySize.y = h;

    m_window->resize(w, h);
}

void Widget::paintGL()
{
    ImGui::SetCurrentContext(m_imgui);
    ImPlot::SetCurrentContext(m_implot);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos({0,0});
	ImGui::SetNextWindowSize(m_io->DisplaySize);
	ImGui::Begin("window", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoResize);

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

Subscriber Widget::subscribeRaw(const std::string& topic, int queue, const RawCb& rawCb, const ros::TransportHints& hints)
{
    auto postEvent = [=](const boost::shared_ptr<const topic_tools::ShapeShifter>& msg){
        QCoreApplication::postEvent(this, new MessageEvent(msg, rawCb));
    };

    ros::Subscriber sub = m_nh.subscribe(topic, queue, boost::function<void(const boost::shared_ptr<const topic_tools::ShapeShifter>&)>(postEvent), {}, hints);
    return Subscriber{std::make_shared<RQTSubscriber>(std::move(sub), this)};
}

bool Widget::event(QEvent* event)
{
    if(event->type() == MessageEvent::Type)
    {
        // Ignore any queued events
        if(!m_running)
            return true;

        reinterpret_cast<MessageEvent*>(event)->call();
        return true;
    }

    return QOpenGLWidget::event(event);
}

void Widget::registerSubscriber(RQTSubscriber* sub)
{
    m_subscribers.push_back(sub);
}

void Widget::deregisterSubscriber(RQTSubscriber* sub)
{
    m_subscribers.erase(std::remove(m_subscribers.begin(), m_subscribers.end(), sub));
}

void Widget::shutdown()
{
    m_running = false;
    for(auto& sub : m_subscribers)
        sub->shutdown();
}

}
