// Plot display
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <imgui_ros/window.h>

#include <imgui_ros/imgui/imgui.h>
#include <imgui_ros/implot/implot.h>
#include <imgui_ros/scrolling_buffer.h>
#include <imgui_ros/topic_selector.h>

#include <pluginlib/class_list_macros.hpp>

#include <std_msgs/msg/float32.hpp>

namespace imgui_plot
{

static constexpr float TIME_WINDOW = 10.0f;

class GUI : public imgui_ros::Window
{
public:
    void initialize() override
    {
        m_node = context()->node();
        m_startTime = m_node->now();
        m_topicSelector = std::make_unique<imgui_ros::TopicSelector>(m_node, std::set<std::string>{"std_msgs/msg/Float32"});
    }

    void paint() override
    {
        float time = (m_node->now() - m_startTime).seconds();

        if(!m_hideTopicSelector)
        {
            ImGui::SetNextItemWidth(-FLT_MIN);
            if(m_topicSelector->draw("##Topic", &m_topic, &m_type))
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

        // Best effort is compatible with both reliable and best effort publishers
        if(m_type == "std_msgs/msg/Float32")
            m_sub = m_node->create_subscription<std_msgs::msg::Float32>(m_topic, rclcpp::SensorDataQoS(), [this](const std_msgs::msg::Float32& msg){ handleFloat32(msg); });
    }

    void handleFloat32(const std_msgs::msg::Float32& msg)
    {
        float data = msg.data;
        m_buffer.push_back((m_node->now() - m_startTime).seconds(), &data);
    }

    rclcpp::Node::SharedPtr m_node;

    bool m_hideTopicSelector = false;
    std::unique_ptr<imgui_ros::TopicSelector> m_topicSelector;

    std::string m_topic;
    std::string m_type;

    imgui_ros::ScrollingBuffer<2048> m_buffer{1};

    rclcpp::Time m_startTime;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_sub;
};

}

PLUGINLIB_EXPORT_CLASS(imgui_plot::GUI, imgui_ros::Window)
