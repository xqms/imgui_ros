// imgui-based image view
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <imgui_ros/window.h>
#include <imgui_ros/topic_selector.h>
#include <imgui_ros/imgui/imgui.h>

#include <pluginlib/class_list_macros.hpp>

#include <ros/callback_queue.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include "decoder.h"

namespace imgui_image_view
{

class ImageView : public imgui_ros::Window
{
public:
    ImageView()
    {
        m_threadNH.setCallbackQueue(&m_threadQueue);

        m_spinner.start();
    }

    ~ImageView()
    {
        m_decoder.shutdown();
        m_spinner.stop();
    }

    void paint() override
    {
        ImGui::SetNextItemWidth(-FLT_MIN);
        if(m_topicSelector.draw("##Topic", &m_topic, &m_type))
            subscribe();

        ros::SteadyTime deadline = ros::SteadyTime::now() + ros::WallDuration{0.001};

        // Fetch all ready frames from decoder
        while(auto newFrame = m_decoder.getNewFrame(deadline))
            m_frame = std::move(newFrame);

        if(m_frame)
        {
            float w = m_frame->width();
            float h = m_frame->height();

            auto avail = ImGui::GetContentRegionAvail();
            float scale = std::min(avail.x / w, avail.y / h);

            ImGui::SetCursorPos({
                ImGui::GetCursorPosX() + (avail.x - scale*w)/2,
                ImGui::GetCursorPosY() + (avail.y - scale*h)/2
            });
            ImGui::Image(reinterpret_cast<void*>(m_frame->texture()), {scale*w, scale*h});
        }
    }

    imgui_ros::Settings getState() const override
    {
        return {
            {"topic", m_topic},
            {"type", m_type}
        };
    }

    void setState(const imgui_ros::Settings& settings) override
    {
        auto topic = settings.get("topic");
        auto type = settings.get("type");
        if(topic && type)
        {
            m_topic = *topic;
            m_type = *type;
            subscribe();
        }
    }

private:
    void subscribe()
    {
        m_sub.shutdown();
        m_frame = {};

        if(m_topic.empty() || m_type.empty())
            return;

        m_decoder.flush();

        if(m_type == "sensor_msgs/Image")
            m_sub = m_threadNH.subscribe(m_topic, 1, &ImageView::handleImage, this);
        else if(m_type == "sensor_msgs/CompressedImage")
            m_sub = m_threadNH.subscribe(m_topic, 1, &ImageView::handleCompressedImage, this);
    }

    void handleImage(const sensor_msgs::ImageConstPtr& msg)
    {
        m_decoder.addMessage(msg);
    }

    void handleCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        m_decoder.addMessage(msg);
    }

    imgui_ros::TopicSelector m_topicSelector{
        "sensor_msgs/Image", "sensor_msgs/CompressedImage"
    };

    std::string m_topic;
    std::string m_type;

    Decoder m_decoder;

    ros::CallbackQueue m_threadQueue;
    ros::NodeHandle m_threadNH;
    ros::AsyncSpinner m_spinner{1, &m_threadQueue};
    ros::Subscriber m_sub;

    std::optional<Decoder::OutputFrame> m_frame;
};

}

PLUGINLIB_EXPORT_CLASS(imgui_image_view::ImageView, imgui_ros::Window)
