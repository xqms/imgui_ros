// Log Viewer
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <imgui_ros/window.h>

#include <imgui_ros/imgui/imgui.h>
#include <imgui_ros/imgui/imgui_internal.h>
#include <imgui_ros/icons.h>
#include <imgui_ros/math.h>

#include <pluginlib/class_list_macros.hpp>

#include <rosgraph_msgs/Log.h>

#include <boost/circular_buffer.hpp>

namespace imgui_log_viewer
{

namespace
{
    void ToggleButton(const char* label, bool* state, const ImVec2& size = ImVec2(0,0))
    {
        ImGui::PushStyleColor(ImGuiCol_Button,
            *state ? ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive)
                   : ImGui::GetStyleColorVec4(ImGuiCol_Button));

        if(ImGui::Button(label, size))
            *state = !*state;

        ImGui::PopStyleColor();
    }
}

class LogViewer : public imgui_ros::Window
{
public:
    void initialize() override
    {
        m_font = context()->loadFont("Monospace", 0.9f);
        if(!m_font)
            throw std::runtime_error{"Could not load monospace font"};

        m_sub = context()->nodeHandle().subscribe("/rosout_agg", 20, &LogViewer::append, this);
    }

    void paint() override
    {
        if(ImGui::BeginTable("toolbar", 2))
        {
            ImGui::TableSetupColumn("filter", ImGuiTableColumnFlags_WidthStretch | ImGuiTableColumnFlags_NoResize);
            ImGui::TableSetupColumn("buttons", ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoResize);

            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(imgui_ros::icon(imgui_ros::Icon::FILTER));
            ImGui::SameLine();
            ImGui::PushItemWidth(-FLT_MIN);

            if(m_filterBuf[0] != 0)
                ImGui::PushStyleColor(ImGuiCol_FrameBg, ImGui::GetColorU32(ImGuiCol_FrameBgActive));
            else
                ImGui::PushStyleColor(ImGuiCol_FrameBg, ImGui::GetColorU32(ImGuiCol_FrameBg));

            ImGui::InputTextWithHint("##filter", "Filter", m_filterBuf, sizeof(m_filterBuf));

            ImGui::PopStyleColor();


            ImGui::TableNextColumn();

            ImGui::SameLine();

            ToggleButton(imgui_ros::icon(imgui_ros::Icon::PAUSE), &m_paused);

            ImGui::SameLine();
            if(ImGui::Button(imgui_ros::icon(imgui_ros::Icon::TRASH)))
                m_entries.clear();

            ImGui::EndTable();
        }

        if(!ImGui::BeginTable("log", 3, ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuterH | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingFixedFit, {-1, -1}))
            return;

        ImGui::PushFont(m_font);

        if(m_defaultWidthNode < 0)
        {
            m_defaultWidthTime = ImGui::CalcTextSize("10:00.00").x;
            m_defaultWidthNode = ImGui::CalcTextSize("/my/test/node/that/is/cool").x;
        }

        ImGui::TableSetupScrollFreeze(0,1);

        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, m_defaultWidthTime);
        ImGui::TableSetupColumn("Node", ImGuiTableColumnFlags_WidthFixed, m_defaultWidthNode);
        ImGui::TableSetupColumn("Message", ImGuiTableColumnFlags_WidthStretch);

        ImGui::TableHeadersRow();

        char timeBuf[256];
        tm timeResult{};

        bool filterActive = m_filterBuf[0] != 0;

        int numEntries = m_entries.size();
        for(int row = 0; row < numEntries; ++row)
        {
            const auto& entry = m_entries[row];

            if(filterActive
                && !std::strstr(entry.msg->msg.c_str(), m_filterBuf)
                && !std::strstr(entry.msg->name.c_str(), m_filterBuf))
                continue;

            bool rowIsHovered = false;

            ImGui::TableNextRow();

            ImGui::TableNextColumn();

            // Rough clipping - if the row is not visible, just push an empty rect with the row height and continue
            if(!ImGui::IsRectVisible({10, entry.height}))
            {
                ImGui::ItemSize({10, entry.height}, 0.0f);
                continue;
            }

            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, 0xff000000);
            ImGui::PushID(row);

            // First column: Timestamp
            auto& stamp = entry.msg->header.stamp;
            std::time_t time = stamp.sec;
            localtime_r(&time, &timeResult);

            std::strftime(timeBuf, sizeof(timeBuf), "%T", &timeResult);

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

            // 2nd column: Node
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(entry.msg->name.c_str());

            if(ImGui::IsItemHovered())
                rowIsHovered = true;

            // 3rd column: Message
            ImGui::TableNextColumn();

            float wrapWidth = ImGui::CalcWrapWidthForPos(ImGui::GetCursorScreenPos(), 0.0f);
            if(wrapWidth != m_logMessageWidth)
            {
                m_logMessageWidth = wrapWidth;
                recalculateHeights();
            }

            // Revisit this when ImGui multiline text edit can do word wrapping
            if constexpr(false)
            {
                ImGui::PushItemWidth(-FLT_MIN);
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, 0.0f));
                ImGui::PushStyleColor(ImGuiCol_FrameBg, 0);
                auto textSize = ImGui::CalcTextSize(entry.msg->msg.c_str(), nullptr, false, ImGui::GetContentRegionAvail().x);
                ImGui::InputTextMultiline("##msg",
                    const_cast<char*>(entry.msg->msg.c_str()),
                    entry.msg->msg.size(),
                    textSize,
                    ImGuiInputTextFlags_ReadOnly
                );
                ImGui::PopStyleColor();
                ImGui::PopStyleVar();
            }
            else
            {
                ImGui::PushTextWrapPos(0);
                ImGui::TextUnformatted(entry.msg->msg.c_str());
                ImGui::PopTextWrapPos();
            }

            if(ImGui::IsItemHovered())
                rowIsHovered = true;

            ImGui::PopStyleColor();

            // Row context menu
            if(ImGui::IsMouseReleased(ImGuiMouseButton_Right) && rowIsHovered)
                ImGui::OpenPopup("context");

            if(ImGui::BeginPopup("context"))
            {
                auto file = std::strrchr(entry.msg->file.c_str(), '/');
                if(file)
                    file = file + 1;
                else
                    file = entry.msg->file.c_str();

                ImGui::BeginDisabled();
                ImGui::Text("%s:%d", file, entry.msg->line);
                ImGui::Text("%s()", entry.msg->function.c_str());
                ImGui::EndDisabled();
                ImGui::Separator();

                char cmdBuf[1024];

                if(ImGui::Selectable("Copy"))
                {
                    if(auto fn = ImGui::GetIO().SetClipboardTextFn)
                        fn(ImGui::GetIO().ClipboardUserData, entry.msg->msg.c_str());
                }
                else if(ImGui::Selectable("Open in Kate"))
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

        ImGui::PopFont();

        if(ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
            ImGui::SetScrollHereY(1.0f);

        ImGui::EndTable();
    }

private:
    struct LogEntry
    {
        rosgraph_msgs::LogConstPtr msg;
        float height = 0;
    };

    void append(const rosgraph_msgs::LogConstPtr& msg)
    {
        if(m_paused)
            return;

        // FIXME: This assumes that timestamps increase monotonically, which might not be the case, especially in networked setups.

        ImGui::PushFont(m_font);

        LogEntry entry;
        entry.msg = msg;
        entry.height = ImGui::CalcTextSize(entry.msg->msg.c_str(), nullptr, false, m_logMessageWidth).y;
        m_entries.push_back(std::move(entry));

        ImGui::PopFont();
    }

    void recalculateHeights()
    {
        ImGui::PushFont(m_font);

        for(auto& entry : m_entries)
            entry.height = ImGui::CalcTextSize(entry.msg->msg.c_str(), nullptr, false, m_logMessageWidth).y;

        ImGui::PopFont();
    }

    ros::Subscriber m_sub;
    boost::circular_buffer<LogEntry> m_entries{256};

    ImFont* m_font = nullptr;

    float m_defaultWidthTime = -1.0f;
    float m_defaultWidthNode = -1.0f;

    char m_filterBuf[1024] = {0};

    bool m_paused = false;

    float m_logMessageWidth = -1.0f;
};

}

PLUGINLIB_EXPORT_CLASS(imgui_log_viewer::LogViewer, imgui_ros::Window)
