// Native imgui GUI
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <imgui_ros/window.h>

#include <ros/ros.h>

#include "../contrib/imgui/backends/imgui_impl_glfw.h"
#include "../contrib/imgui/backends/imgui_impl_opengl3.h"

#include <implot.h>

#include <GLFW/glfw3.h>

#include <fontconfig/fontconfig.h>

#include <pluginlib/class_loader.hpp>
#include <topic_tools/shape_shifter.h>

#include <random>
#include <filesystem>
#include <regex>

#include <boost/program_options.hpp>


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
    std::uint64_t instanceID = 0;

    std::string pluginName;

    boost::shared_ptr<imgui_ros::Window> plugin;

private:
    ros::NodeHandle& m_nh;
};

int main(int argc, char** argv)
{
    namespace po = boost::program_options;
    namespace fs = std::filesystem;

    ros::init(argc, argv, "gui", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    po::options_description desc("Options");

    desc.add_options()
        ("help,h", "This help message")
        ("perspective,p", po::value<std::string>()->default_value("default"), "The perspective to load")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if(vm.count("help"))
    {
        std::cout << "Usage: gui [options]\n";
        std::cout << desc << "\n";
        return 0;
    }

    po::notify(vm);

    // Load settings
    fs::path configHome = [](){
        if(auto env = getenv("XDG_CONFIG_HOME"))
            return fs::path(env);
        else
            return fs::path(getenv("HOME")) / fs::path(".config");
    }();
    fs::path configDir = configHome / "ros.org" / "imgui_ros";

    if(!fs::exists(configDir))
        fs::create_directories(configDir);

    std::string perspective = vm["perspective"].as<std::string>();
    fs::path imguiConfigPath = configDir / (perspective + ".imgui.ini");
    fs::path configPath = configDir / (perspective + ".ini");

    ClassLoader loader{"imgui_ros", "imgui_ros::Window"};

    // Setup window
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    constexpr int MSAA = 4;
    glfwWindowHint(GLFW_SAMPLES, MSAA);

    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(1280, 720, "imgui_ros", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    glEnable(GL_MULTISAMPLE);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();

    io.IniFilename = nullptr;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    if(fs::exists(imguiConfigPath))
        ImGui::LoadIniSettingsFromDisk(imguiConfigPath.c_str());

    // Font
    std::string fontFile;
    {
        FcConfig* config = FcInitLoadConfigAndFonts();

        FcPattern* pat = FcNameParse((const FcChar8*)"Noto Sans");
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
        float scaleX = 1.0f, scaleY = 1.0f;
        GLFWmonitor* const monitor = glfwGetPrimaryMonitor();
        glfwGetMonitorContentScale(monitor, &scaleX, &scaleY);

        io.Fonts->AddFontFromFileTTF(fontFile.c_str(), std::round(scaleX * 15.0f), NULL, NULL);
    }

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

    // Load settings
    if(fs::exists(configPath))
    {
        std::ifstream in(configPath);
        std::string line;

        static const std::regex REGEX_SECTION{R"EOS(^\[(.*)##([a-fA-F\d]+)\]$)EOS"};
        static const std::regex REGEX_PLUGIN{R"EOS(^plugin=\"(.*)\"$)EOS"};
        static const std::regex REGEX_SETTING{R"EOS(^setting_(.*)="(.*)"$)EOS"};
        static const std::regex REGEX_WINDOW{R"EOS(^window=(\d+)x(\d+)\+(\d+)\+(\d+)$)EOS"};

        std::string windowTitle;
        std::uint64_t id;
        std::string plugin;
        imgui_ros::Settings settings;

        int windowWidth = -1;
        int windowHeight = -1;
        int windowX = -1;
        int windowY = -1;

        auto instantiate = [&](){
            if(plugin.empty())
            {
                ROS_ERROR("Window '%s' does not contain a plugin key, ignoring plugin!", windowTitle.c_str());
                return;
            }

            auto w = std::make_unique<Window>(nh);
            w->windowTitle = windowTitle;
            w->instanceID = id;

            w->pluginName = plugin;
            w->plugin = loader.createInstance(plugin);
            if(!w->plugin)
            {
                ROS_ERROR("Could not load plugin '%s', ignoring", plugin.c_str());
                return;
            }

            w->plugin->initialize(w.get());
            w->windowTitle = windowTitle;

            w->plugin->setState(settings);

            ROS_DEBUG("Loaded window '%s##%lu'", w->windowTitle.c_str(), w->instanceID);

            windows.push_back(std::move(w));

            windowTitle = {};
        };

        while(std::getline(in, line))
        {
            if(line.empty())
                continue;

            std::smatch matchData;
            if(std::regex_match(line, matchData, REGEX_SECTION))
            {
                if(!windowTitle.empty())
                    instantiate();

                windowTitle = matchData[1].str();
                id = std::strtoull(matchData[2].str().c_str(), nullptr, 16);
                plugin = {};
                settings.clear();
            }
            else if(std::regex_match(line, matchData, REGEX_PLUGIN))
            {
                plugin = matchData[1].str();
            }
            else if(std::regex_match(line, matchData, REGEX_SETTING))
            {
                settings[matchData[1].str()] = matchData[2].str();
            }
            else if(std::regex_match(line, matchData, REGEX_WINDOW))
            {
                windowWidth = std::atoi(matchData[1].str().c_str());
                windowHeight = std::atoi(matchData[2].str().c_str());
                windowX = std::atoi(matchData[3].str().c_str());
                windowY = std::atoi(matchData[4].str().c_str());
            }
            else
            {
                ROS_ERROR("Invalid settings line: '%s'", line.c_str());
                std::exit(1);
            }
        }

        if(!windowTitle.empty())
            instantiate();

        if(windowWidth > 0)
        {
            glfwSetWindowPos(window, windowX, windowY);
            glfwSetWindowSize(window, windowWidth, windowHeight);
        }
    }

    while(ros::ok())
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
                        w->pluginName = cls;

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

        char buf[1024];
        for(auto it = windows.begin(); it != windows.end(); )
        {
            auto& w = *it;
            bool open = true;

            ImGui::PushID(w->instanceID);

            snprintf(buf, sizeof(buf), "%s##%lx", w->windowTitle.c_str(), w->instanceID);

            if(ImGui::Begin(buf, &open))
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

        if(glfwWindowShouldClose(window))
        {
            ImGui::OpenPopup("Close");

            // Always center this window when appearing
            ImVec2 center = ImGui::GetMainViewport()->GetCenter();
            ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

            bool open = true;
            if(ImGui::BeginPopupModal("Close", &open, ImGuiWindowFlags_AlwaysAutoResize))
            {
                ImGui::TextUnformatted("Save perspective?");

                if(ImGui::Button("Yes", ImVec2(80, 0)))
                {
                    ImGui::SaveIniSettingsToDisk(imguiConfigPath.c_str());

                    std::ofstream out{configPath};

                    int x, y, width, height;
                    glfwGetWindowPos(window, &x, &y);
                    glfwGetWindowSize(window, &width, &height);
                    out << "window=" << width << "x" << height << "+" << x << "+" << y << "\n\n";

                    for(auto& w : windows)
                    {
                        out << "[" << w->windowTitle << "##" << std::hex << w->instanceID << "]\n";
                        out << "plugin=\"" << w->pluginName << "\"\n";

                        auto settings = w->plugin->getState();
                        for(auto& pair : settings)
                            out << "setting_" << pair.first << "=\"" << pair.second << "\"\n";

                        out << "\n";
                    }

                    break;
                }

                ImGui::SetItemDefaultFocus();
                ImGui::SameLine();

                if(ImGui::Button("No", ImVec2(80, 0)))
                {
                    break;
                }

                ImGui::SameLine();

                if(ImGui::Button("Cancel", ImVec2(80, 0)))
                {
                    ImGui::CloseCurrentPopup();
                    glfwSetWindowShouldClose(window, false);
                }

                ImGui::EndPopup();
            }

            if(!open)
            {
                glfwSetWindowShouldClose(window, false);
                break;
            }
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
