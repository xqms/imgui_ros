// Wrench display
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <imgui_ros/window.h>

#include <imgui_ros/imgui/imgui.h>
#include <imgui_ros/implot/implot.h>
#include <imgui_ros/scrolling_buffer.h>
#include <imgui_ros/topic_selector.h>

#include <pluginlib/class_list_macros.hpp>

#include <geometry_msgs/WrenchStamped.h>

namespace imgui_wrench
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

        if(ImPlot::BeginPlot("Force", {-1.0f, ImGui::GetContentRegionAvail().y/2.0f}))
        {
            ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels | ImPlotAxisFlags_NoMenus | ImPlotAxisFlags_NoLabel;
            ImPlot::SetupAxes("Time", "Force [N]", flags, ImPlotAxisFlags_NoLabel);

            ImPlot::SetupAxisLimits(ImAxis_X1,
                time - TIME_WINDOW,
                time,
                ImGuiCond_Always
            );

            ImPlot::SetupAxisLimits(ImAxis_Y1, -15, 15, ImGuiCond_Always);

            ImPlot::SetNextLineStyle(ImGui::ColorConvertU32ToFloat4(0xFF5284DD));
            ImPlot::PlotLine("X", m_buffer.timeData(), m_buffer.rowData(0), m_buffer.size(), m_buffer.offset());

            ImPlot::SetNextLineStyle(ImGui::ColorConvertU32ToFloat4(0xFF68A855));
            ImPlot::PlotLine("Y", m_buffer.timeData(), m_buffer.rowData(1), m_buffer.size(), m_buffer.offset());

            ImPlot::SetNextLineStyle(ImGui::ColorConvertU32ToFloat4(0xFFB0724C));
            ImPlot::PlotLine("Z", m_buffer.timeData(), m_buffer.rowData(2), m_buffer.size(), m_buffer.offset());

            ImPlot::EndPlot();
        }

        if(ImPlot::BeginPlot("Torque", {-1.0f, -1.0f}))
        {
            ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels | ImPlotAxisFlags_NoMenus | ImPlotAxisFlags_NoLabel;
            ImPlot::SetupAxes("Time", "Torque [Nm]", flags, ImPlotAxisFlags_NoLabel);

            ImPlot::SetupAxisLimits(ImAxis_X1,
                time - TIME_WINDOW,
                time,
                ImGuiCond_Always
            );

            ImPlot::SetupAxisLimits(ImAxis_Y1, -6, 6, ImGuiCond_Always);

            ImPlot::SetNextLineStyle(ImGui::ColorConvertU32ToFloat4(0xFF5284DD));
            ImPlot::PlotLine("X", m_buffer.timeData(), m_buffer.rowData(3), m_buffer.size(), m_buffer.offset());

            ImPlot::SetNextLineStyle(ImGui::ColorConvertU32ToFloat4(0xFF68A855));
            ImPlot::PlotLine("Y", m_buffer.timeData(), m_buffer.rowData(4), m_buffer.size(), m_buffer.offset());

            ImPlot::SetNextLineStyle(ImGui::ColorConvertU32ToFloat4(0xFFB0724C));
            ImPlot::PlotLine("Z", m_buffer.timeData(), m_buffer.rowData(5), m_buffer.size(), m_buffer.offset());

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

        if(m_type == "geometry_msgs/WrenchStamped")
            m_sub = context()->nodeHandle().subscribe(m_topic, 1, &GUI::handleWrenchStamped, this);
        else if(m_type == "geometry_msgs/Wrench")
            m_sub = context()->nodeHandle().subscribe(m_topic, 1, &GUI::handleWrench, this);
    }

    void handleWrench(const geometry_msgs::Wrench& msg)
    {
        float data[6] = {
            static_cast<float>(msg.force.x),
            static_cast<float>(msg.force.y),
            static_cast<float>(msg.force.z),
            static_cast<float>(msg.torque.x),
            static_cast<float>(msg.torque.y),
            static_cast<float>(msg.torque.z)
        };
        m_buffer.push_back((ros::Time::now() - m_startTime).toSec(), data);
    }
    void handleWrenchStamped(const geometry_msgs::WrenchStamped& msg)
    {
        float data[6] = {
            static_cast<float>(msg.wrench.force.x),
            static_cast<float>(msg.wrench.force.y),
            static_cast<float>(msg.wrench.force.z),
            static_cast<float>(msg.wrench.torque.x),
            static_cast<float>(msg.wrench.torque.y),
            static_cast<float>(msg.wrench.torque.z)
        };
        m_buffer.push_back((msg.header.stamp - m_startTime).toSec(), data);
    }

    bool m_hideTopicSelector = false;
    imgui_ros::TopicSelector m_topicSelector{
        "geometry_msgs/WrenchStamped",
        "geometry_msgs/Wrench"
    };

    std::string m_topic;
    std::string m_type;

    imgui_ros::ScrollingBuffer<2048> m_buffer{6};

    ros::Time m_startTime = ros::Time::now();

    ros::Subscriber m_sub;
};

}

PLUGINLIB_EXPORT_CLASS(imgui_wrench::GUI, imgui_ros::Window)
