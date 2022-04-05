// Log Viewer
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <imgui_ros/window.h>

#include <imgui_ros/imgui/imgui.h>
#include <imgui_ros/math.h>

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

        ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, 0xff000000);

        char timeBuf[256];
        tm timeResult{};

        for(std::size_t row = 0; row < m_entries.size(); ++row)
        {
            const auto& entry = m_entries[row];

            bool rowIsHovered = false;

            ImGui::TableNextRow();

            ImGui::PushID(row);

            ImGui::TableNextColumn();

            auto& stamp = entry.msg->header.stamp;
            std::time_t time = stamp.sec;
            localtime_r(&time, &timeResult);

            std::strftime(timeBuf, sizeof(timeBuf), "%R", &timeResult);

            std::uint32_t color = [&](){
                switch(entry.msg->level)
                {
                    case rosgraph_msgs::Log::DEBUG: return 0xff9cbc1a;
                    case rosgraph_msgs::Log::INFO:  return 0xffffffff;
                    case rosgraph_msgs::Log::WARN:  return 0xff0074f6;
                    case rosgraph_msgs::Log::ERROR: return 0xff1515ed;
                    case rosgraph_msgs::Log::FATAL: return 0xff1515ed;
                    default: return 0xffb6599b;
                }
            }();

            ImGui::PushStyleColor(ImGuiCol_Text, color);

            ImGui::Text("%s.%02llu", timeBuf, stamp.nsec / 10000000ULL);
            if(ImGui::IsItemHovered())
                rowIsHovered = true;

            ImGui::TableNextColumn();
            ImGui::TextUnformatted(entry.msg->name.c_str());

            if(ImGui::IsItemHovered())
                rowIsHovered = true;

            ImGui::TableNextColumn();
            ImGui::TextUnformatted(entry.msg->msg.c_str());

            if(ImGui::IsItemHovered())
                rowIsHovered = true;

            ImGui::PopStyleColor();

            if(ImGui::IsMouseReleased(ImGuiMouseButton_Right) && rowIsHovered)
                ImGui::OpenPopup("context");

            if(ImGui::BeginPopup("context"))
            {
                char cmdBuf[1024];

                if(ImGui::Selectable("Open in Kate"))
                {
                    snprintf(cmdBuf, sizeof(cmdBuf), "kate '%s:%d' &", entry.msg->file.c_str(), entry.msg->line);
                    if(system(cmdBuf))
                        ;
                }
                else if(ImGui::Selectable("Open in KDevelop"))
                {
                    snprintf(cmdBuf, sizeof(cmdBuf), "kdevelop '%s:%d' &", entry.msg->file.c_str(), entry.msg->line);
                    if(system(cmdBuf))
                        ;
                }

                ImGui::EndPopup();
            }

            ImGui::PopID();
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
