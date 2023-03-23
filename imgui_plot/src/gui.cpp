// Plot display
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <imgui_ros/window.h>

#include <imgui_ros/imgui/imgui.h>
#include <imgui_ros/implot/implot.h>
#include <imgui_ros/scrolling_buffer.h>
#include <imgui_ros/topic_selector.h>

#include <pluginlib/class_list_macros.hpp>

#include <std_msgs/Float32.h>

namespace imgui_plot
{

static constexpr float TIME_WINDOW = 10.0f;

class GUI : public imgui_ros::Window
{
public:
    void initialize() override
    {
    }

    void paint() override
    {
        ros::Time now = ros::Time::now();
        float time = (now - m_startTime).toSec();

        if(!m_hideTopicSelector)
        {
            ImGui::SetNextItemWidth(-FLT_MIN);
            if(m_topicSelector.draw("##Topic", &m_topic, &m_type))
                subscribe();
        }

        if(ImPlot::BeginPlot("Value", {-1.0f, -1.0f}, ImPlotFlags_NoTitle | ImPlotFlags_NoLegend))
        {
            ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels | ImPlotAxisFlags_NoMenus | ImPlotAxisFlags_NoLabel;
            ImPlot::SetupAxes("Time", "Value", flags, ImPlotAxisFlags_NoLabel);

            ImPlot::SetupAxisLimits(ImAxis_X1,
                time - TIME_WINDOW,
                time,
                ImGuiCond_Always
            );

            ImPlot::SetupAxisLimits(ImAxis_Y1, -0.2, 1.2, ImGuiCond_Always);

            ImPlot::SetNextLineStyle(ImGui::ColorConvertU32ToFloat4(0xFF5284DD));
            ImPlot::PlotLine("x", m_buffer.timeData(), m_buffer.rowData(0), m_buffer.size(), m_buffer.offset());

            ImPlot::EndPlot();
        }
    }

    void setState(const imgui_ros::Settings& settings) override
    {
        if(auto val = settings.get("type"))
            m_type = *val;
        if(auto val = settings.get("topic"))
            m_topic = *val;

        subscribe();
    }

    imgui_ros::Settings getState() const override
    {
        return {
            {"type", m_type},
            {"topic", m_topic}
        };
    }

private:
    void subscribe()
    {
        m_sub = {};

        if(m_topic.empty())
            return;

        if(m_type == "std_msgs/Float32")
            m_sub = context()->nodeHandle().subscribe(m_topic, 1, &GUI::handleFloat32, this);
    }

    void handleFloat32(const std_msgs::Float32& msg)
    {
        float data = msg.data;
        m_buffer.push_back((ros::Time::now() - m_startTime).toSec(), &data);
    }

    bool m_hideTopicSelector = false;
    imgui_ros::TopicSelector m_topicSelector{
        "std_msgs/Float32"
    };

    std::string m_topic;
    std::string m_type;

    imgui_ros::ScrollingBuffer<2048> m_buffer{1};

    ros::Time m_startTime = ros::Time::now();

    ros::Subscriber m_sub;
};

}

PLUGINLIB_EXPORT_CLASS(imgui_plot::GUI, imgui_ros::Window)
