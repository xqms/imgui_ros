// Wrench display
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <imgui_ros/window.h>

#include <imgui_ros/imgui/imgui.h>
#include <imgui_ros/implot/implot.h>
#include <imgui_ros/scrolling_buffer.h>
#include <imgui_ros/topic_selector.h>

#include <pluginlib/class_list_macros.hpp>

#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace imgui_wrench
{

static constexpr float TIME_WINDOW = 10.0f;

class GUI : public imgui_ros::Window
{
public:
    void initialize() override
    {
        m_node = context()->node();
        m_startTime = m_node->now();
        m_topicSelector = std::make_unique<imgui_ros::TopicSelector>(m_node, std::set<std::string>{
            "geometry_msgs/msg/WrenchStamped",
            "geometry_msgs/msg/Wrench"
        });
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

        // Best effort is compatible with both reliable and best effort publishers
        if(m_type == "geometry_msgs/msg/WrenchStamped")
            m_sub = m_node->create_subscription<geometry_msgs::msg::WrenchStamped>(m_topic, rclcpp::SensorDataQoS(), [this](const geometry_msgs::msg::WrenchStamped& msg){ handleWrenchStamped(msg); });
        else if(m_type == "geometry_msgs/msg/Wrench")
            m_sub = m_node->create_subscription<geometry_msgs::msg::Wrench>(m_topic, rclcpp::SensorDataQoS(), [this](const geometry_msgs::msg::Wrench& msg){ handleWrench(msg); });
    }

    void handleWrench(const geometry_msgs::msg::Wrench& msg)
    {
        float data[6] = {
            static_cast<float>(msg.force.x),
            static_cast<float>(msg.force.y),
            static_cast<float>(msg.force.z),
            static_cast<float>(msg.torque.x),
            static_cast<float>(msg.torque.y),
            static_cast<float>(msg.torque.z)
        };
        m_buffer.push_back((m_node->now() - m_startTime).seconds(), data);
    }
    void handleWrenchStamped(const geometry_msgs::msg::WrenchStamped& msg)
    {
        float data[6] = {
            static_cast<float>(msg.wrench.force.x),
            static_cast<float>(msg.wrench.force.y),
            static_cast<float>(msg.wrench.force.z),
            static_cast<float>(msg.wrench.torque.x),
            static_cast<float>(msg.wrench.torque.y),
            static_cast<float>(msg.wrench.torque.z)
        };
        m_buffer.push_back((rclcpp::Time{msg.header.stamp} - m_startTime).seconds(), data);
    }

    rclcpp::Node::SharedPtr m_node;

    bool m_hideTopicSelector = false;
    std::unique_ptr<imgui_ros::TopicSelector> m_topicSelector;

    std::string m_topic;
    std::string m_type;

    imgui_ros::ScrollingBuffer<2048> m_buffer{6};

    rclcpp::Time m_startTime;

    rclcpp::SubscriptionBase::SharedPtr m_sub;
};

}

PLUGINLIB_EXPORT_CLASS(imgui_wrench::GUI, imgui_ros::Window)
