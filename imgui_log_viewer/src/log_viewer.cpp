// Log Viewer
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <imgui_ros/window.h>

#include <imgui_ros/imgui/imgui.h>

#include <pluginlib/class_list_macros.hpp>

#include <rosgraph_msgs/Log.h>

#include <boost/circular_buffer.hpp>

namespace imgui_log_viewer
{

class LogViewer : public imgui_ros::Window
{
public:
    void initialize() override
    {
        m_font = context()->loadFont("Monospace");

        m_sub = context()->nodeHandle().subscribe("/rosout_agg", 20, &LogViewer::append, this);
    }

    void paint() override
    {
        ImGui::PushFont(m_font);

        ImGui::BeginTable("log", 3, ImGuiTableFlags_BordersV | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingFixedFit, {-1, -1});

        if(m_defaultWidthNode < 0)
        {
            m_defaultWidthTime = ImGui::CalcTextSize("10:00.00").x;
            m_defaultWidthNode = ImGui::CalcTextSize("/my/test/node/that/is/cool").x;
        }

        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, m_defaultWidthTime);
        ImGui::TableSetupColumn("Node", ImGuiTableColumnFlags_WidthFixed, m_defaultWidthNode);
        ImGui::TableSetupColumn("Message", ImGuiTableColumnFlags_WidthStretch);

        ImGui::TableHeadersRow();

        char timeBuf[256];
        tm timeResult{};

        for(auto& entry : m_entries)
        {
            ImGui::TableNextRow();

            ImGui::TableNextColumn();

            auto& stamp = entry.msg->header.stamp;
            std::time_t time = stamp.sec;
            localtime_r(&time, &timeResult);

            std::strftime(timeBuf, sizeof(timeBuf), "%R", &timeResult);

            ImGui::Text("%s.%02llu", timeBuf, stamp.nsec / 10000000ULL);

            ImGui::TableNextColumn();
            ImGui::TextUnformatted(entry.msg->name.c_str());

            ImGui::TableNextColumn();
            ImGui::TextUnformatted(entry.msg->msg.c_str());
        }

        ImGui::EndTable();

        ImGui::PopFont();
    }

private:
    struct LogEntry
    {
        rosgraph_msgs::LogConstPtr msg;
        float height = 0;
    };

    void append(const rosgraph_msgs::LogConstPtr& msg)
    {
        LogEntry entry;
        entry.msg = msg;
        entry.height = ImGui::CalcTextSize(msg->msg.c_str()).y;
        m_entries.push_back(std::move(entry));
    }

    ros::Subscriber m_sub;
    boost::circular_buffer<LogEntry> m_entries{1024};

    ImFont* m_font = nullptr;

    float m_defaultWidthTime = -1.0f;
    float m_defaultWidthNode = -1.0f;
};

}

PLUGINLIB_EXPORT_CLASS(imgui_log_viewer::LogViewer, imgui_ros::Window)
