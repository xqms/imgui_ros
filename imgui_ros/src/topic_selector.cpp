// Topic selector combobox
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <imgui_ros/topic_selector.h>

#include <imgui.h>

#include <ros/master.h>

namespace imgui_ros
{

TopicSelector::TopicSelector(const std::set<std::string>& types) noexcept
 : m_types{types}
{
}

TopicSelector::~TopicSelector() noexcept
{
}

bool TopicSelector::draw(const char* label, std::string* selectedTopic, std::string* selectedType)
{
    bool topicChanged = false;

    if(ImGui::BeginCombo(label, selectedTopic->c_str()))
    {
        // If the combo box has just been opened, update the list of topics
        if(ImGui::IsWindowAppearing())
            updateTopics();

        // Combo box entries
        for(auto& t : m_topics)
        {
            bool selected = (t.topic == *selectedTopic);
            if(ImGui::Selectable(t.topic.c_str(), selected))
            {
                *selectedTopic = t.topic;
                if(selectedType)
                    *selectedType = t.type;
                topicChanged = true;
            }
            if(selected)
                ImGui::SetItemDefaultFocus();
        }

        ImGui::EndCombo();
    }

    return topicChanged;
}

void TopicSelector::updateTopics()
{
    std::vector<ros::master::TopicInfo> topics;
    ros::master::getTopics(topics);

    m_topics.clear();
    for(auto& topic : topics)
    {
        if(m_types.count(topic.datatype) != 0)
            m_topics.push_back({topic.name, topic.datatype});
    }
}


}
