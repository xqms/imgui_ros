// imgui-based image view
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef IMGUI_DEFINE_MATH_OPERATORS
#define IMGUI_DEFINE_MATH_OPERATORS
#include <GL/gl.h>
#endif

#include <imgui_ros/window.h>
#include <imgui_ros/topic_selector.h>
#include <imgui_ros/imgui/imgui.h>
#include <imgui_ros/imgui/imgui_internal.h>
#include <imgui_ros/scrolling_buffer.h>
#include <imgui_ros/nfd/nfd.h>

#include <pluginlib/class_list_macros.hpp>

#include <ros/callback_queue.h>
#include <ros/names.h>

#include <rosfmt/full.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>

#include <png.h>

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

        if(m_camInfo && m_correctAspectRatio)
        {
            auto camInfo = m_camInfo;

            float fx = m_camInfo->K[0];
            float fy = m_camInfo->K[3*1 + 1];
            float factor = fx / fy;

            h *= factor;
        }

        if(m_rotation == 90 || m_rotation == -90)
            std::swap(w,h);

        auto uvCoords = uvCoordsForRotation(m_rotation);

        if(ImGui::BeginChild("imgwin", ImGui::GetContentRegionAvail(), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
        {
            auto avail = ImGui::GetContentRegionAvail();
            float scale = m_fillScreen
                ? std::max(avail.x / w, avail.y / h)
                : std::min(avail.x / w, avail.y / h);

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
                    m_frame->texture(),
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

                {
                    int step = 1;
                    ImGui::SetNextItemWidth(200);
                    ImGui::InputScalar("Rate limit", ImGuiDataType_U32, &m_rateLimit, &step);
                }

                ImGui::Checkbox("Hide topic selector", &m_hideTopicSelector);

                ImGui::Checkbox("Show FPS", &m_showFPS);
                ImGui::Checkbox("Fill screen", &m_fillScreen);
                ImGui::Checkbox("Correct aspect ratio", &m_correctAspectRatio);
                ImGui::SetNextItemWidth(200);
                ImGui::InputText("Heartbeat topic", m_heartbeatTopic, sizeof(m_heartbeatTopic));
                if(ImGui::IsItemDeactivated())
                    initHeartbeat();

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

                ImGui::Text("%dx%d", m_frame->width(), m_frame->height());

                if(ImGui::Button("Save image"))
                {
                    saveImage();
                    ImGui::CloseCurrentPopup();
                }

                ImGui::EndPopup();
            }

            if(m_showFPS)
            {
                auto bottomRight = ImGui::GetContentRegionMax();
                auto height = ImGui::GetFontSize();
                ImGui::SetCursorPos({bottomRight.x - 200, bottomRight.y - height});
                ImGui::PlotHistogram("##hist", m_rateBuffer.rowData(1), m_rateBuffer.size(), m_rateBuffer.offset(), fpsText, 0.0f, 1.5f, {200,height});
            }

            if(m_heartbeatTopic[0] != 0 && rateNow > 1)
            {
                std_msgs::Header msg;
                msg.stamp = ros::Time::now();
                m_pub_heartBeat.publish(msg);
            }

        }
        ImGui::EndChild();
    }

    imgui_ros::Settings getState() const override
    {
        return {
            {"topic", m_topic},
            {"type", m_type},
            {"hide_topic_selector", std::to_string(m_hideTopicSelector)},
            {"rotation", std::to_string(m_rotation)},
            {"correct_aspect_ratio", std::to_string(m_correctAspectRatio)},
            {"fill_screen", std::to_string(m_fillScreen)},
            {"show_fps", std::to_string(m_showFPS)},
            {"rate_limit", std::to_string(m_rateLimit)},
            {"heartbeat_topic", m_heartbeatTopic}
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

        if(auto v = settings.get("correct_aspect_ratio"))
            m_correctAspectRatio = (*v == "1");

        if(auto v = settings.get("fill_screen"))
            m_fillScreen = (*v == "1");

        if(auto v = settings.get("show_fps"))
            m_showFPS = (*v == "1");

        if(auto v = settings.get("rate_limit"))
            m_rateLimit = std::atoi(v->c_str());

        if(auto v = settings.get("heartbeat_topic"))
        {
            strncpy(m_heartbeatTopic, v->c_str(), sizeof(m_heartbeatTopic) -1);
            m_heartbeatTopic[sizeof(m_heartbeatTopic)-1] = 0;
            initHeartbeat();
        }
    }

    void saveImage()
    {
        if(!m_frame)
            return;

        glBindTexture(GL_TEXTURE_2D, m_frame->texture());

        int size_x = 0;
        int size_y = 0;

        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &size_x);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &size_y);

        ROSFMT_INFO("Capturing image with resolution {}x{}", size_x, size_y);

        const std::size_t dataSize = size_x*size_y*3;

        std::vector<uint8_t> data(dataSize);

        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glPixelStorei(GL_PACK_ROW_LENGTH, size_x);
        glPixelStorei(GL_PACK_IMAGE_HEIGHT, size_y);
        glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
        glPixelStorei(GL_PACK_SKIP_ROWS, 0);
        glPixelStorei(GL_PACK_SKIP_IMAGES, 0);

        glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, data.data());

        saveImageData(data, size_x, size_y);
    }

private:

    void saveImageData(const std::vector<uint8_t>& data, int size_x, int size_y)
    {
        nfdu8char_t* outPath = {};
        nfdu8filteritem_t filters[] = { {"PNG image", "png"} };
        nfdsavedialogu8args_t args = {};
        args.filterList = filters;
        args.filterCount = 1;

        context()->setupNFDHandle(&args.parentWindow);

        auto result = NFD_SaveDialogU8_With(&outPath, &args);
        if(result == NFD_OKAY)
        {
            FILE* f = fopen(outPath, "w");
            if(!f)
            {
                ROSFMT_ERROR("Could not write to {}", outPath);
                NFD_FreePathU8(outPath);
                return;
            }

            auto png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);

            if(!png)
                throw std::runtime_error{"Could not create PNG writer"};

            auto pngInfo = png_create_info_struct(png);
            if(!pngInfo)
            {
                png_destroy_write_struct(&png, nullptr);
                throw std::runtime_error{"Could not create PNG writer"};
            }

            if(setjmp(png_jmpbuf(png)))
            {
                png_destroy_write_struct(&png, &pngInfo);
                throw std::runtime_error{"PNG error"};
            }

            png_init_io(png, f);

            png_set_IHDR(png, pngInfo, size_x, size_y, 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

            std::vector<const uint8_t*> rows(size_y);
            for(int y = 0; y < size_y; ++y)
                rows[y] = &data[y*size_x*3];

            png_set_rows(png, pngInfo, const_cast<uint8_t**>(rows.data()));

            png_write_png(png, pngInfo, PNG_TRANSFORM_IDENTITY, NULL);

            png_destroy_write_struct(&png, &pngInfo);

            fclose(f);

            ROSFMT_INFO("Saved to {}", outPath);

            NFD_FreePathU8(outPath);
        }
    }

    void subscribe()
    {
        m_sub.shutdown();
        m_frame = {};

        if(m_topic.empty() || m_type.empty())
            return;

        m_decoder.flush();
        m_camInfo = {};

        if(m_type == "sensor_msgs/Image")
            m_sub = m_threadNH.subscribe(m_topic, 1, &ImageView::handleImage, this);
        else if(m_type == "sensor_msgs/CompressedImage")
            m_sub = m_threadNH.subscribe(m_topic, 1, &ImageView::handleCompressedImage, this);

        std::string infoTopic = ros::names::parentNamespace(m_topic) + "/camera_info";
        m_sub_camInfo = m_threadNH.subscribe(infoTopic, 1, &ImageView::handleCameraInfo, this);
    }

    void initHeartbeat()
    {
        if(m_heartbeatTopic[0] != 0)
            m_pub_heartBeat = m_threadNH.advertise<std_msgs::Header>(m_heartbeatTopic, 1);
        else
            m_pub_heartBeat = {};
    }

    bool computeFrameSkip(const ros::Time& stamp)
    {
        if(m_paused)
            return true;

        m_rateEstimator.put(stamp);
        m_messageCounter++;

        ros::Time now = ros::Time::now();
        if(m_lastMsgTime == ros::Time(0))
            m_lastMsgTime = now;
        else
        {
            // Basic token bucket algorithm for rate limiting
            ros::Duration elapsed = now - m_lastMsgTime;
            m_lastMsgTime = now;

            m_throttleAllowance = std::min(2.0f,
                m_throttleAllowance + static_cast<float>(elapsed.toSec()) * m_rateLimit
            );

            if(m_throttleAllowance < 1.0f)
                return true;

            m_throttleAllowance -= 1.0f;
        }

        return false;
    }

    void handleImage(const sensor_msgs::ImageConstPtr& msg)
    {
        if(computeFrameSkip(msg->header.stamp))
            return;

        m_decoder.addMessage(msg);
    }

    void handleCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        if(computeFrameSkip(msg->header.stamp))
            return;

        m_decoder.addMessage(msg);
    }

    void handleCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        m_camInfo = msg;
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
    ros::Subscriber m_sub_camInfo;
    ros::Publisher m_pub_heartBeat;

    std::optional<Decoder::OutputFrame> m_frame;

    bool m_paused = false;
    bool m_hideTopicSelector = false;
    int m_rotation = 0;

    RateEstimator m_rateEstimator;
    imgui_ros::ScrollingBuffer<200> m_rateBuffer{2};
    std::atomic_uint64_t m_messageCounter = 0;

    sensor_msgs::CameraInfoConstPtr m_camInfo;
    bool m_correctAspectRatio = false;

    bool m_showFPS = true;
    bool m_fillScreen = false;
    char m_heartbeatTopic[256] = {0};

    int m_rateLimit = 60;
    float m_throttleAllowance = 0.0f;
    ros::Time m_lastMsgTime;
};

}

PLUGINLIB_EXPORT_CLASS(imgui_image_view::ImageView, imgui_ros::Window)
