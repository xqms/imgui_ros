// Native imgui GUI
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <imgui_ros/window.h>

#include <ros/ros.h>

#include "../contrib/imgui/backends/imgui_impl_glfw.h"
#include "../contrib/imgui/backends/imgui_impl_opengl3.h"

#include <implot.h>

#include <GLFW/glfw3.h>

#include <pluginlib/class_loader.hpp>
#include <topic_tools/shape_shifter.h>

#include <random>

using ClassLoader = pluginlib::ClassLoader<imgui_ros::Window>;

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

class SubscriberImpl : public imgui_ros::Subscriber::Impl
{
public:
    explicit SubscriberImpl(const ros::Subscriber& sub)
     : m_sub{sub}
    {}

    virtual ~SubscriberImpl()
    {}

    int getNumPublishers() override
    { return m_sub.getNumPublishers(); }

    void shutdown() override
    { m_sub.shutdown(); }

private:
    ros::Subscriber m_sub;
};

class Window : public imgui_ros::Context
{
public:
    explicit Window(ros::NodeHandle& nh)
     : m_nh{nh}
    {}

    virtual ~Window()
    {}

    ros::NodeHandle& nodeHandle() override
    { return m_nh; }

    void setUpdateRate(float updateRate) override
    {}

    void setWindowTitle(const std::string& title) override
    {
        windowTitle = title;
    }

    imgui_ros::Subscriber subscribeRaw(const std::string& topic, int queue, const RawCb& rawCb, const ros::TransportHints& hints = {}) override
    {
        ros::Subscriber sub = m_nh.subscribe(topic, queue, boost::function<void(const boost::shared_ptr<const topic_tools::ShapeShifter>&)>{rawCb}, {}, hints);

        return imgui_ros::Subscriber{std::make_shared<SubscriberImpl>(sub)};
    }

    std::string windowTitle = "plugin";
    boost::shared_ptr<imgui_ros::Window> plugin;
    std::uint64_t instanceID = 0;

private:
    ros::NodeHandle& m_nh;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gui", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    ClassLoader loader{"imgui_ros", "imgui_ros::Window"};

    // Setup window
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(1280, 720, "imgui_ros", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();

    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Implot
    ImPlot::CreateContext();

    std::mt19937 mt(0);
    std::uniform_int_distribution<std::uint64_t> instanceIDGenerator;

    std::vector<std::unique_ptr<Window>> windows;

    while (ros::ok() && !glfwWindowShouldClose(window))
    {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        glfwPollEvents();

        ros::spinOnce();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::DockSpaceOverViewport();

        if(ImGui::BeginMainMenuBar())
        {
            if(ImGui::BeginMenu("Plugins"))
            {
                auto classes = loader.getDeclaredClasses();

                for(auto& cls : classes)
                {
                    if(ImGui::MenuItem(cls.c_str()))
                    {
                        // Load plugin
                        auto w = std::make_unique<Window>(nh);
                        w->instanceID = instanceIDGenerator(mt);
                        w->plugin = loader.createInstance(cls);

                        if(!w->plugin)
                            ImGui::OpenPopup("PluginError");
                        else
                        {
                            w->plugin->initialize(w.get());
                            windows.push_back(std::move(w));
                        }
                    }
                }

                if(ImGui::BeginPopupModal("PluginError"))
                {
                    ImGui::TextUnformatted("Could not load plugin. Please check terminal for error message.");
                    ImGui::EndPopup();
                }

                ImGui::EndMenu();
            }

            ImGui::EndMainMenuBar();
        }


        for(auto it = windows.begin(); it != windows.end(); )
        {
            auto& w = *it;
            bool open = true;

            ImGui::PushID(w->instanceID);

            if(ImGui::Begin((w->windowTitle + "##" + std::to_string(w->instanceID)).c_str(), &open))
            {
                w->plugin->paint();
            }

            ImGui::End();

            ImGui::PopID();

            if(open)
                ++it;
            else
                it = windows.erase(it);
        }

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0, 0, 0, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
