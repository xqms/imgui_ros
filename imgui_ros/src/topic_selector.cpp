// Topic selector combobox
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <imgui_ros/topic_selector.h>

#include <ranges>

#include <imgui.h>

#include <rclcpp/node.hpp>

namespace imgui_ros
{

TopicSelector::TopicSelector(const rclcpp::Node::SharedPtr& node, const std::set<std::string>& types) noexcept
 : m_node{node}, m_types{types}
{
}

TopicSelector::~TopicSelector() noexcept
{
}

bool TopicSelector::draw(const char* label, std::string* selectedTopic, std::string* selectedType)
{
    bool topicChanged = false;

    bool showType = m_types.size() != 1;

    if(ImGui::BeginCombo(label, selectedTopic->c_str()))
    {
        // If the combo box has just been opened, update the list of topics
        if(ImGui::IsWindowAppearing())
            updateTopics();

        ImGui::BeginTable("topics", showType ? 2 : 1, ImGuiTableFlags_NoSavedSettings | ImGuiTableFlags_SizingStretchProp);
        ImGui::TableSetupColumn("topic", ImGuiTableColumnFlags_WidthStretch);

        if(showType)
            ImGui::TableSetupColumn("type", ImGuiTableColumnFlags_WidthStretch);

        for(auto& t : m_topics)
        {
            bool selected = (t.topic == *selectedTopic);

            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            if(ImGui::Selectable(t.topic.c_str(), selected, ImGuiSelectableFlags_SpanAllColumns))
            {
                *selectedTopic = t.topic;
                if(selectedType)
                    *selectedType = t.type;
                topicChanged = true;
            }
            if(selected)
                ImGui::SetItemDefaultFocus();

            if(showType)
            {
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(t.type.c_str());
            }
        }

        ImGui::EndTable();

        ImGui::EndCombo();
    }

    return topicChanged;
}

void TopicSelector::updateTopics()
{
    auto info = m_node->get_topic_names_and_types();

    m_topics.clear();
    for(auto& topic : info)
    {
        auto type = [&]() -> std::optional<std::string> {
            for(auto& t : topic.second)
            {
                if(m_types.count(t) != 0)
                    return t;
            }
            return {};
        }();
        if(!type)
            continue;

        m_topics.push_back({topic.first, *type});
    }
    std::sort(m_topics.begin(), m_topics.end());
}


}
