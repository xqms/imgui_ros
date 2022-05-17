// imgui-based image view
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef IMGUI_DEFINE_MATH_OPERATORS
#define IMGUI_DEFINE_MATH_OPERATORS
#endif

#include <imgui_ros/window.h>
#include <imgui_ros/topic_selector.h>
#include <imgui_ros/imgui/imgui.h>
#include <imgui_ros/imgui/imgui_internal.h>
#include <imgui_ros/scrolling_buffer.h>

#include <pluginlib/class_list_macros.hpp>

#include <ros/callback_queue.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include "decoder.h"
#include "cuda_decoder.h"
#include "rate_estimator.h"

namespace imgui_image_view
{

namespace
{
    struct UVCoords
    {
        ImVec2 topLeft;
        ImVec2 bottomLeft;
        ImVec2 bottomRight;
        ImVec2 topRight;
    };

    UVCoords uvCoordsForRotation(int angle)
    {
        switch(angle)
        {
            case 0:   return {{0.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 0.0f}};
            case 90:  return {{1.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f}};
            case 180: return {{1.0f, 1.0f}, {1.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 1.0f}};
            case -90: return {{0.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 0.0f}, {0.0f, 0.0f}};
        }

        throw std::logic_error{"Invalid rotation angle"};
    }
}

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
        if(!m_hideTopicSelector)
        {
            ImGui::SetNextItemWidth(-FLT_MIN);
            if(m_topicSelector.draw("##Topic", &m_topic, &m_type))
                subscribe();
        }

        // Fetch all ready frames from decoder
        while(auto newFrame = m_decoder.getNewFrame())
            m_frame = std::move(newFrame);

        if(!m_frame)
            return;

        float w = m_frame->width();
        float h = m_frame->height();

        if(m_rotation == 90 || m_rotation == -90)
            std::swap(w,h);

        auto avail = ImGui::GetContentRegionAvail();
        float scale = std::min(avail.x / w, avail.y / h);

        auto uvCoords = uvCoordsForRotation(m_rotation);

        ImVec2 size{scale*w, scale*h};
        ImGui::SetCursorPos({
            ImGui::GetCursorPosX() + (avail.x - scale*w)/2,
            ImGui::GetCursorPosY() + (avail.y - scale*h)/2
        });
        auto id = ImGui::GetID("image");

        ImGuiWindow* window = ImGui::GetCurrentWindow();
        const ImRect bb(window->DC.CursorPos, window->DC.CursorPos + size);
        ImGui::ItemSize(bb);
        if(ImGui::ItemAdd(bb, id))
        {
            window->DrawList->AddImageQuad(
                reinterpret_cast<void*>(m_frame->texture()),
                bb.GetTL(), bb.GetBL(), bb.GetBR(), bb.GetTR(),
                uvCoords.topLeft, uvCoords.bottomLeft, uvCoords.bottomRight, uvCoords.topRight
            );
        }

        uint64_t msgs = m_messageCounter.exchange(0);
        float rateNow = m_rateEstimator.rateNow();
        float bufData[2] = {rateNow, float(msgs)};
        m_rateBuffer.push_back(0, bufData);

        char fpsText[256];
        snprintf(fpsText, sizeof(fpsText), "%.1f", rateNow);

        if(ImGui::BeginPopupContextItem())
        {
            ImGui::Checkbox("Pause", &m_paused);
            ImGui::Checkbox("Hide topic selector", &m_hideTopicSelector);

            int step = 90;
            ImGui::SetNextItemWidth(200);
            ImGui::InputScalar("Rotation",  ImGuiDataType_S32, &m_rotation, &step, nullptr, "%d°");
            while(m_rotation < -179)
                m_rotation += 360;
            while(m_rotation > 180)
                m_rotation -= 360;

            ImGui::PlotHistogram("Messages", m_rateBuffer.rowData(1), m_rateBuffer.size(), m_rateBuffer.offset(), nullptr, 0.0f, 1.5f, {200,0});

            ImGui::PlotHistogram("FPS", m_rateBuffer.rowData(0), m_rateBuffer.size(), m_rateBuffer.offset(),
                fpsText,
                0.0f, FLT_MAX, {200,0}
            );

            ImGui::EndPopup();
        }

        {
            auto bottomRight = ImGui::GetContentRegionMax();
            auto height = ImGui::GetFontSize();
            ImGui::SetCursorPos({bottomRight.x - 200, bottomRight.y - height});
            ImGui::PlotHistogram("##hist", m_rateBuffer.rowData(1), m_rateBuffer.size(), m_rateBuffer.offset(), fpsText, 0.0f, 1.5f, {200,height});
        }
    }

    imgui_ros::Settings getState() const override
    {
        return {
            {"topic", m_topic},
            {"type", m_type},
            {"hide_topic_selector", std::to_string(m_hideTopicSelector)},
            {"rotation", std::to_string(m_rotation)}
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

        if(auto b = settings.get("hide_topic_selector"))
            m_hideTopicSelector = (*b == "1");

        if(auto v = settings.get("rotation"))
            m_rotation = std::atoi(v->c_str());
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
        if(m_paused)
            return;

        m_decoder.addMessage(msg);
        m_rateEstimator.put(msg->header.stamp);
        m_messageCounter--;
    }

    void handleCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        if(m_paused)
            return;

        m_decoder.addMessage(msg);
        m_rateEstimator.put(msg->header.stamp);
        m_messageCounter++;
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

    bool m_paused = false;
    bool m_hideTopicSelector = false;
    int m_rotation = 0;

    RateEstimator m_rateEstimator;
    imgui_ros::ScrollingBuffer<200> m_rateBuffer{2};
    std::atomic_uint64_t m_messageCounter = 0;
};

}

PLUGINLIB_EXPORT_CLASS(imgui_image_view::ImageView, imgui_ros::Window)
