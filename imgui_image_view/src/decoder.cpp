// Decoder with texture output
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

extern "C"
{
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glx.h>
}

#include "decoder.h"

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
#include <libavutil/opt.h>
}

#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <fstream>
#include <utility>

#include <rosfmt/full.h>

namespace
{
    std::string safePixelFormatName(AVPixelFormat fmt)
    {
        auto name = av_get_pix_fmt_name(fmt);
        return name ? name : std::to_string(fmt);
    }

    class Task
    {
    public:
        Task()
        {
        }

        std::unique_ptr<imgui_image_view::Decoder::OutputFrameData> frameData;
    };

    class Texture
    {
    public:
        Texture()
        {
        }

        Texture(int w, int h)
        {
            glCreateTextures(GL_TEXTURE_2D, 1, &m_texture);

            glTextureParameteri(m_texture, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTextureParameteri(m_texture, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTextureParameteri(m_texture, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
            glTextureParameteri(m_texture, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);

            const float borderColor[3] = {0.5f, 0.5f, 0.5f};
            glTextureParameterfv(m_texture, GL_TEXTURE_BORDER_COLOR, borderColor);

            glTextureParameteri(m_texture, GL_TEXTURE_MAX_LEVEL, 1);

            glTextureStorage2D(m_texture, 1, GL_RGBA8, w, h);
        }

        Texture(const Texture&) = delete;
        Texture& operator=(const Texture&) = delete;

        Texture(Texture&& other)
         : m_texture{other.m_texture}
        {
            other.m_texture = -1;
        }

        Texture& operator=(Texture&& other)
        {
            std::swap(m_texture, other.m_texture);
            return *this;
        }

        ~Texture()
        {
            if(m_texture != static_cast<GLuint>(-1))
                glDeleteTextures(1, &m_texture);
        }

        GLuint texture()
        { return m_texture; }

    private:
        GLuint m_texture = static_cast<GLuint>(-1);
    };
}

namespace imgui_image_view
{

class Decoder::OutputFrameData
{
public:
    OutputFrameData(Decoder::Private* priv)
     : decoder{priv}
    {
    }

    ~OutputFrameData()
    {
        glDeleteSync(fence);
    }

    void prepare(int requestedWidth, int requestedHeight);

    Decoder::Private* decoder;

    Texture texture;
    int width = 0;
    int height = 0;
    GLsync fence{};

    sensor_msgs::CompressedImageConstPtr msg;
    sensor_msgs::ImageConstPtr uncompressed;
    ros::WallDuration timeSpentDecoding;
};

class Decoder::Private
{
public:
    explicit Private(GLXContext glxContext);
    ~Private();

    void returnBuffer(std::unique_ptr<Decoder::OutputFrameData>&& buffer);

    bool addMessage(const sensor_msgs::CompressedImageConstPtr& msg);
    bool addMessage(const sensor_msgs::ImageConstPtr& msg);
    std::optional<OutputFrame> getNewFrame(const ros::SteadyTime& deadline);

    void flush();

private:
    bool initializeDecoder(const std::string& encoding, int width = -1, int height = -1);
    void decodePacket(AVPacket* packet);
    void pushFrame(AVFrame* frame, AVColorSpace colorSpace = AVCOL_SPC_RGB, AVColorRange colorRange = AVCOL_RANGE_UNSPECIFIED);

    std::deque<Task> m_outputQueue;

    std::mutex m_jobMutex;
    std::condition_variable m_jobOutCond;
    std::atomic_bool m_shutdown = false;

    std::deque<std::unique_ptr<OutputFrameData>> m_freeOutFrames;
    std::mutex m_freeOutMutex;
    std::condition_variable m_freeOutCond;

    Display* m_display = {};
    GLXPbuffer m_pbuffer = {};
    GLXContext m_context = {};

    AVCodecContext* m_codecCtx{};
    AVCodecParserContext* m_parserCtx{};
    std::string m_codecName;

    SwsContext* m_swsContext{};
    int m_swsWidth = -1;
    int m_swsHeight = -1;
    int m_swsFormat = -1;

    bool m_contextCurrent = false;
};


void Decoder::OutputFrameData::prepare(int requestedWidth, int requestedHeight)
{
    if(fence)
    {
        glDeleteSync(fence);
        fence = nullptr;
    }

    if(width == requestedWidth && height == requestedHeight)
        return;

    texture = Texture{requestedWidth, requestedHeight};

    width = requestedWidth;
    height = requestedHeight;
}


Decoder::Private::Private(GLXContext glxContext)
{
    // Create output buffers
    for(unsigned int i = 0; i < 5; ++i)
    {
        m_freeOutFrames.push_back(std::make_unique<OutputFrameData>(this));
    }

    m_display = XOpenDisplay(nullptr);

    /* Choose config */
    int configCount = 0;
    constexpr static const int fbAttributes[] = { None };
    GLXFBConfig* configs = glXChooseFBConfig(m_display, DefaultScreen(m_display), fbAttributes, &configCount);
    if(!configCount)
    {
        fprintf(stderr, "Platform::WindowlessGlxContext: no supported framebuffer configuration found\n");
        std::exit(1);
    }

    /* Create pbuffer */
    constexpr static const int pbufferAttributes[] = {
        GLX_PBUFFER_WIDTH,  32,
        GLX_PBUFFER_HEIGHT, 32,
        None
    };
    m_pbuffer = glXCreatePbuffer(m_display, configs[0], pbufferAttributes);

    /* Get pointer to proper context creation function */
    const PFNGLXCREATECONTEXTATTRIBSARBPROC glXCreateContextAttribsARB = reinterpret_cast<PFNGLXCREATECONTEXTATTRIBSARBPROC>(glXGetProcAddress(reinterpret_cast<const GLubyte*>("glXCreateContextAttribsARB")));

    GLint contextAttributes[11] = {
        GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
        GLX_CONTEXT_MINOR_VERSION_ARB, 1,
        GLX_CONTEXT_PROFILE_MASK_ARB, GLX_CONTEXT_CORE_PROFILE_BIT_ARB,
        0
    };

    m_context = glXCreateContextAttribsARB(m_display, configs[0], glxContext, True, contextAttributes);

    if(!m_context)
        throw std::runtime_error{"Could not create GLX context"};

    XFree(configs);
}

Decoder::Private::~Private()
{
    if(m_swsContext)
        sws_freeContext(m_swsContext);

    if(m_codecCtx)
        avcodec_free_context(&m_codecCtx);

    if(m_parserCtx)
    {
        av_parser_close(m_parserCtx);
        m_parserCtx = nullptr;
    }

    if(m_context)
        glXDestroyContext(m_display, m_context);
    if(m_pbuffer)
        glXDestroyPbuffer(m_display, m_pbuffer);
    if(m_display)
        XCloseDisplay(m_display);
}

bool Decoder::Private::initializeDecoder(const std::string& encoding, int width, int height)
{
    if(!m_contextCurrent)
    {
        glXMakeContextCurrent(m_display, m_pbuffer, m_pbuffer, m_context);
        m_contextCurrent = true;
    }

    if(!encoding.empty() && m_codecCtx && m_codecName == encoding)
        return true;

    if(encoding.empty())
        return true;

    if(m_codecCtx)
        avcodec_free_context(&m_codecCtx);

    if(m_parserCtx)
    {
        av_parser_close(m_parserCtx);
        m_parserCtx = nullptr;
    }

    AVCodecID codecID;
    AVPixelFormat pixelFormat = AV_PIX_FMT_NONE;

    if(strcasecmp(encoding.c_str(), "h264") == 0)
        codecID = AV_CODEC_ID_H264;
    else if(strcasecmp(encoding.c_str(), "h264") == 0)
        codecID = AV_CODEC_ID_H265;
    else if(strcasecmp(encoding.c_str(), "bgr8; jpeg compressed bgr8") == 0)
        codecID = AV_CODEC_ID_MJPEG;
    else
    {
        fprintf(stderr, "Unsupported encoding: '%s'\n", encoding.c_str());
        return false;
    }

    AVCodec* codec = avcodec_find_decoder(codecID);
    if(!codec)
        throw std::runtime_error{"Could not find decoder"};

    m_parserCtx = av_parser_init(codecID);
    if(!m_parserCtx)
        throw std::runtime_error{"Could not allocate parser"};

    m_codecCtx = avcodec_alloc_context3(codec);
    if(!m_codecCtx)
        throw std::runtime_error{"Could not allocate codec context"};

    m_codecCtx->flags |= AV_CODEC_FLAG_LOW_DELAY | AV_CODEC_FLAG_OUTPUT_CORRUPT;
    m_codecCtx->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;

    if(codecID == AV_CODEC_ID_MJPEG)
        m_parserCtx->flags |= PARSER_FLAG_COMPLETE_FRAMES;

    if(pixelFormat != AV_PIX_FMT_NONE)
        m_codecCtx->pix_fmt = pixelFormat;

    if(width >= 0)
        m_codecCtx->width = width;
    if(height >= 0)
        m_codecCtx->height = height;

    if(avcodec_open2(m_codecCtx, codec, nullptr) < 0)
        throw std::runtime_error{"Could not open codec"};

    ROSFMT_INFO("Initialized codec with settings: pix_fmt={}, sw_pix_fmt={}, color_range={}",
        safePixelFormatName(m_codecCtx->pix_fmt),
        safePixelFormatName(m_codecCtx->sw_pix_fmt),
        m_codecCtx->color_range
    );

    m_codecName = encoding;

    return true;
}

void Decoder::Private::decodePacket(AVPacket* packet)
{
    if(avcodec_send_packet(m_codecCtx, packet) < 0)
    {
        ROSFMT_WARN_NAMED("decoder", "Could not send packet to decoder");
        return;
    }

    AVFrame* frame = av_frame_alloc();

    int ret = 0;
    while(ret >= 0)
    {
        ret = avcodec_receive_frame(m_codecCtx, frame);
        if(ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            return;
        else if(ret < 0)
        {
            ROSFMT_WARN_NAMED("decoder", "Error during decoding");
            return;
        }

        pushFrame(frame, m_codecCtx->colorspace, m_codecCtx->color_range);
    }

    av_frame_free(&frame);
}

void Decoder::Private::pushFrame(AVFrame* frame, AVColorSpace colorSpace, AVColorRange colorRange)
{
    int dataWidth = frame->width;
    int dataHeight = frame->height;

    // Obtain a texture we can copy to
    // NOTE: We cannot get rid of this copy, since it changes the
    //  pixel format to the internal one of the GPU.

    std::unique_ptr<OutputFrameData> frameData = {};
    {
        std::unique_lock<std::mutex> lock(m_freeOutMutex);

        while(!m_shutdown && m_freeOutFrames.empty())
        {
            ROSFMT_DEBUG_THROTTLE_NAMED(1.0, "decoder", "Decoder has to wait for free output frames");
            m_freeOutCond.wait(lock);
        }

        if(m_shutdown)
            return;

        frameData = std::move(m_freeOutFrames.front());
        m_freeOutFrames.pop_front();
    }

    frameData->prepare(dataWidth, dataHeight);

    AVFrame* targetFrame = {};

    if(frame->format == AV_PIX_FMT_RGB32 || frame->format == AV_PIX_FMT_RGBA || frame->format == AV_PIX_FMT_RGB0)
    {
        ROSFMT_INFO("no conversion");
    }
    else
    {
        // We use and set color space / range information below, so we can
        // map the old deprecated AV_PIX_FMT_YUVJ420P to its general
        // variant AV_PIX_FMT_YUV420P to avoid a warning
        if(frame->format == AV_PIX_FMT_YUVJ420P)
            frame->format = AV_PIX_FMT_YUV420P;

        // Use sws to convert to target pixfmt
        if(m_swsContext && (m_swsFormat != frame->format || m_swsWidth != dataWidth || m_swsHeight != dataHeight))
        {
            sws_freeContext(m_swsContext);
            m_swsContext = nullptr;
        }

        if(!m_swsContext)
        {
            m_swsContext = sws_alloc_context();

            av_opt_set_int(m_swsContext, "srcw", dataWidth, 0);
            av_opt_set_int(m_swsContext, "srch", dataHeight, 0);
            av_opt_set_int(m_swsContext, "src_format", frame->format, 0);
            av_opt_set_int(m_swsContext, "dstw", dataWidth, 0);
            av_opt_set_int(m_swsContext, "dsth", dataHeight, 0);
            av_opt_set_int(m_swsContext, "dst_format", AV_PIX_FMT_RGBA, 0);

            if(sws_init_context(m_swsContext, nullptr, nullptr))
            {
                ROSFMT_ERROR("Could not create libswscale context for conversion from {} to RGBA",
                    safePixelFormatName(static_cast<AVPixelFormat>(frame->format))
                );
                return;
            }

            if(colorRange != AVCOL_RANGE_UNSPECIFIED)
            {
                int in_full, out_full, brightness, contrast, saturation;
                const int *inv_table, *table;

                sws_getColorspaceDetails(m_swsContext,
                    (int **)&inv_table, &in_full,
                    (int **)&table, &out_full,
                    &brightness, &contrast, &saturation
                );

                int range = (colorRange == AVCOL_RANGE_JPEG) ? 1 : 0;
                const int* tbl = sws_getCoefficients(colorSpace);
                sws_setColorspaceDetails(m_swsContext, tbl, range, tbl, 0, brightness, contrast, saturation);
            }
        }

        targetFrame = av_frame_alloc();
        targetFrame->width = frame->width;
        targetFrame->height = frame->height;
        targetFrame->format = AV_PIX_FMT_RGBA;
        if(auto err = av_frame_get_buffer(targetFrame, 0))
        {
            ROSFMT_ERROR("Could not allocate buffer: {}", err);
            return;
        }

        if(sws_scale(m_swsContext, frame->data, frame->linesize, 0, frame->height, targetFrame->data, targetFrame->linesize) <= 0)
        {
            ROSFMT_ERROR("Could not convert pixel format using swscale");
            return;
        }

        frame = targetFrame;
    }

    glTextureSubImage2D(
        frameData->texture.texture(),
        0, // level
        0, 0, // offset
        dataWidth, dataHeight, // size
        GL_RGBA, GL_UNSIGNED_BYTE,
        frame->data[0]
    );

    // Provide sync fence
    frameData->fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    glFlush();

    // Mark as finished
    {
        std::unique_lock<std::mutex> lock{m_jobMutex};
        Task out;
        out.frameData = std::move(frameData);
        m_outputQueue.push_back(std::move(out));
        m_jobOutCond.notify_one();
    }

    if(targetFrame)
        av_frame_free(&targetFrame);
}

void Decoder::Private::returnBuffer(std::unique_ptr<Decoder::OutputFrameData>&& buffer)
{
    buffer->msg.reset();

    std::unique_lock<std::mutex> lock(m_freeOutMutex);
    m_freeOutFrames.push_back(std::move(buffer));
    m_freeOutCond.notify_one();
}

bool Decoder::Private::addMessage(const sensor_msgs::CompressedImageConstPtr& msg)
{
    if(!initializeDecoder(msg->format))
        return false;

    const uint8_t* dataPtr = msg->data.data();
    std::size_t remaining = msg->data.size();

    AVPacket* packet = av_packet_alloc();

    while(remaining > 0)
    {
        int ret = av_parser_parse2(
            m_parserCtx, m_codecCtx, &packet->data, &packet->size,
            dataPtr, remaining, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0
        );

        if(ret < 0)
        {
            ROSFMT_WARN_NAMED("decoder", "Error during packet parsing");
            break;
        }

        dataPtr += ret;
        remaining -= ret;

        if(packet->size > 0)
            decodePacket(packet);
    }

    av_packet_free(&packet);

    return true;
}

bool Decoder::Private::addMessage(const sensor_msgs::ImageConstPtr& msg)
{
    if(!initializeDecoder({}, msg->width, msg->height))
        return false;

    AVPixelFormat format = {};
    if(msg->encoding == "rgb8")
        format = AV_PIX_FMT_RGB24;
    else if(msg->encoding == "bgr8")
        format = AV_PIX_FMT_BGR24;
    else if(msg->encoding == "mono8")
        format = AV_PIX_FMT_GRAY8;
    else
    {
        ROSFMT_WARN_THROTTLE(1.0, "Received image with unsupported encoding '{}', ignoring", msg->encoding);
        return false;
    }

    AVFrame* frame = av_frame_alloc();

    frame->width = msg->width;
    frame->height = msg->height;
    frame->data[0] = const_cast<uint8_t*>(msg->data.data());
    frame->linesize[0] = msg->step;
    frame->format = format;

    pushFrame(frame);

    av_frame_free(&frame);

    return true;
}

void Decoder::Private::flush()
{
    std::unique_lock<std::mutex> lock(m_jobMutex);

    for(auto& frame : m_outputQueue)
        returnBuffer(std::move(frame.frameData));

    m_outputQueue.clear();
}

std::optional<Decoder::OutputFrame> Decoder::Private::getNewFrame(const ros::SteadyTime& deadline)
{
    std::unique_lock<std::mutex> lock(m_jobMutex);

    std::chrono::steady_clock::time_point tp(std::chrono::nanoseconds(deadline.toNSec()));

    m_jobOutCond.wait_until(lock, tp, [&](){
        return !m_outputQueue.empty();
    });

    if(m_outputQueue.empty())
        return {};

    auto& task = m_outputQueue.front();

    // Task is done, but the final GL transfer from buffer into texture might
    // still be running.

    ros::SteadyTime now = ros::SteadyTime::now();
    uint64_t wait_ns = 0;
    if(now < deadline)
        wait_ns = (deadline - now).toNSec();

    GLenum syncStatus = glClientWaitSync(task.frameData->fence, 0, wait_ns);
    if(syncStatus == GL_TIMEOUT_EXPIRED)
        return {}; // Keep task in queue

    if(syncStatus != GL_ALREADY_SIGNALED && syncStatus != GL_CONDITION_SATISFIED)
    {
        ROSFMT_ERROR("glClientWaitSync() error");
        std::abort();
    }

    OutputFrame outputFrame(std::move(task.frameData));

    m_outputQueue.pop_front();

    return std::move(outputFrame);
}



Decoder::OutputFrame::OutputFrame(std::unique_ptr<OutputFrameData>&& data)
 : m_frameData{std::move(data)}
{
}

Decoder::OutputFrame::OutputFrame(OutputFrame&& other)
 : m_frameData{std::move(other.m_frameData)}
{
}

Decoder::OutputFrame& Decoder::OutputFrame::operator=(OutputFrame&& other)
{
    if(m_frameData)
    {
        m_frameData->decoder->returnBuffer(std::move(m_frameData));
    }

    m_frameData = std::move(other.m_frameData);

    return *this;
}

Decoder::OutputFrame::~OutputFrame()
{
    if(m_frameData)
        m_frameData->decoder->returnBuffer(std::move(m_frameData));

    m_frameData.release();
}

GLuint Decoder::OutputFrame::texture()
{
    return m_frameData->texture.texture();
}

int Decoder::OutputFrame::width() const
{
    return m_frameData->width;
}

int Decoder::OutputFrame::height() const
{
    return m_frameData->height;
}

const sensor_msgs::CompressedImageConstPtr& Decoder::OutputFrame::message() const
{
    return m_frameData->msg;
}

ros::WallDuration Decoder::OutputFrame::timeSpentDecoding() const
{
    return m_frameData->timeSpentDecoding;
}









Decoder::Decoder()
 : m_d{std::make_unique<Private>(glXGetCurrentContext())}
{
}

Decoder::~Decoder() = default;

bool Decoder::addMessage(const sensor_msgs::CompressedImageConstPtr& img)
{
    return m_d->addMessage(img);
}

bool Decoder::addMessage(const sensor_msgs::ImageConstPtr& img)
{
    return m_d->addMessage(img);
}

std::optional<Decoder::OutputFrame> Decoder::getNewFrame(const ros::SteadyTime& deadline)
{
    return m_d->getNewFrame(deadline);
}

void Decoder::flush()
{
    m_d->flush();
}

}
