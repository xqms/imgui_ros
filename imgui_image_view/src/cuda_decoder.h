// High-performance image decoder with texture output
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef IMGUI_IMAGE_VIEW_CUDA_DECODER_H
#define IMGUI_IMAGE_VIEW_CUDA_DECODER_H

#if CUDA_DECODER

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <GL/gl.h>

namespace imgui_image_view
{

class Decoder
{
public:
    class OutputFrameData;

    class OutputFrame
    {
    public:
        explicit OutputFrame(std::unique_ptr<OutputFrameData>&& data);
        ~OutputFrame();

        OutputFrame(OutputFrame&& other);
        OutputFrame& operator=(OutputFrame&& other);

        OutputFrame(const OutputFrame&) = delete;
        OutputFrame& operator=(const OutputFrame&) = delete;

        const sensor_msgs::CompressedImageConstPtr& message() const;
        GLuint texture();
        ros::WallDuration timeSpentDecoding() const;
        int width() const;
        int height() const;

    private:
        std::unique_ptr<OutputFrameData> m_frameData;
    };

    explicit Decoder();
    ~Decoder();

    Decoder(const Decoder&) = delete;
    Decoder& operator=(const Decoder&) = delete;

    /**
     * @brief Insert image into decoding queue
     *
     * This method is thread-safe.
     **/
    bool addMessage(const sensor_msgs::CompressedImageConstPtr& img);
    bool addMessage(const sensor_msgs::ImageConstPtr& img);

    /**
     * @brief Get output frame
     *
     * Returns the next output frame. If no frame is ready yet, this method
     * returns an empty Optional immediately.
     *
     * This method is thread-safe.
     **/
    std::optional<OutputFrame> getNewFrame(const ros::SteadyTime& deadline = {});

    void flush();

    void shutdown();

private:
    class Private;

    std::unique_ptr<Private> m_d;
};

}

#endif
#endif
