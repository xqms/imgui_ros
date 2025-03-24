// Decoder with texture output
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#if CUDA_DECODER

extern "C"
{
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glx.h>
}

#include "cuda_decoder.h"

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include <nppi_color_conversion.h>
#include <nppi_data_exchange_and_initialization.h>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavutil/pixdesc.h>
#include <libavutil/hwcontext_cuda.h>
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

    class CUDATexture
    {
    public:
        CUDATexture()
        {
        }

        CUDATexture(int w, int h, std::optional<cudaStream_t> stream)
         : m_stream{stream}
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

            if(stream)
            {
                if(auto err = cudaGraphicsGLRegisterImage(&m_resource, m_texture, GL_TEXTURE_2D, cudaGraphicsRegisterFlagsWriteDiscard))
                {
                    fprintf(stderr, "Could not register GL texture with CUDA: %s (%d)\n", cudaGetErrorString(err), err);
                    std::abort();
                }
            }
        }

        CUDATexture(const CUDATexture&) = delete;
        CUDATexture& operator=(const CUDATexture&) = delete;

        CUDATexture(CUDATexture&& other)
         : m_stream{other.m_stream}
         , m_texture{other.m_texture}
         , m_resource{other.m_resource}
         , m_levelArray{other.m_levelArray}
         , m_mapped{other.m_mapped}
        {
            other.m_texture = -1;
            other.m_resource = 0;
            other.m_levelArray = {};
            other.m_mapped = false;
        }

        CUDATexture& operator=(CUDATexture&& other)
        {
            std::swap(m_texture, other.m_texture);
            std::swap(m_stream, other.m_stream);
            std::swap(m_mapped, other.m_mapped);
            std::swap(m_resource, other.m_resource);
            std::swap(m_levelArray, other.m_levelArray);
            return *this;
        }

        ~CUDATexture()
        {
            if(m_mapped)
                unmap();

            if(m_resource)
                cudaGraphicsUnregisterResource(m_resource);

            if(m_texture != static_cast<GLuint>(-1))
                glDeleteTextures(1, &m_texture);
        }

        void map()
        {
            assert(!m_mapped);

            if(auto err = cudaGraphicsMapResources(1, &m_resource, *m_stream))
            {
                fprintf(stderr, "Could not map GL buffer to CUDA: %s (%d)\n", cudaGetErrorString(err), err);
                std::abort();
            }

            cudaMipmappedArray_t array{}; // NOTE: I assume I don't have to free this...
            if(auto err = cudaGraphicsResourceGetMappedMipmappedArray(&array, m_resource))
            {
                fprintf(stderr, "Could not get mapped array: %s (%d)\n", cudaGetErrorString(err), err);
                std::abort();
            }

            if(auto err = cudaGetMipmappedArrayLevel(&m_levelArray, array, 0))
            {
                fprintf(stderr, "Could not get mapped array: %s (%d)\n", cudaGetErrorString(err), err);
                std::abort();
            }

            m_mapped = true;
        }

        void unmap()
        {
            assert(m_mapped);

            if(auto err = cudaGraphicsUnmapResources(1, &m_resource, *m_stream))
            {
                fprintf(stderr, "Could not unmap GL buffer from CUDA: %s (%d)\n", cudaGetErrorString(err), err);
                std::abort();
            }

            m_mapped = false;
        }

        cudaArray_t deviceArray()
        { return m_levelArray; }

        GLuint texture()
        { return m_texture; }

    private:
        std::optional<cudaStream_t> m_stream{};
        GLuint m_texture = static_cast<GLuint>(-1);
        cudaGraphicsResource_t m_resource{};
        cudaArray_t m_levelArray{};
        bool m_mapped = false;
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

        if(devBuffer)
            cudaFree(&devBuffer);
        if(devBuffer2)
            cudaFree(&devBuffer2);
    }

    void prepare(int requestedWidth, int requestedHeight, const std::optional<cudaStream_t>& stream);

    Decoder::Private* decoder;

    CUDATexture texture;
    int width = 0;
    int height = 0;
    uint8_t* devBuffer = 0;
    uint8_t* devBuffer2 = 0;
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
    std::optional<OutputFrame> getNewFrame();

    void flush();

    void shutdown();

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

    int m_cudaDevice = -1;

    Display* m_display = {};
    GLXPbuffer m_pbuffer = {};
    GLXContext m_context = {};

    cudaStream_t m_stream = {};
    CUcontext m_cudaCtx{};
    NppStreamContext m_nppCtx{};

    AVCodecContext* m_codecCtx{};
    AVCodecParserContext* m_parserCtx{};
    std::string m_codecName;

    AVBufferRef* m_hwFramesCtx{};
    bool m_contextCurrent = false;
};


void Decoder::OutputFrameData::prepare(int requestedWidth, int requestedHeight, const std::optional<cudaStream_t>& stream)
{
    if(fence)
    {
        glDeleteSync(fence);
        fence = nullptr;
    }

    if(width == requestedWidth && height == requestedHeight)
        return;

    if(devBuffer)
        cudaFree(&devBuffer);

    texture = CUDATexture{requestedWidth, requestedHeight, stream};

    if(stream)
    {
        if(cudaMalloc(&devBuffer, 4*requestedWidth*requestedHeight))
            throw std::runtime_error{"Out of GPU memory."};
        if(cudaMalloc(&devBuffer2, 4*requestedWidth*requestedHeight))
            throw std::runtime_error{"Out of GPU memory."};

        cudaMemset(devBuffer, 0xFF, 4*requestedWidth*requestedHeight);
    }

    width = requestedWidth;
    height = requestedHeight;
}


Decoder::Private::Private(GLXContext glxContext)
{
    [&](){
        unsigned int numDevices = 0;
        std::array<int, 10> devices;
        if(auto err = cudaGLGetDevices(&numDevices, devices.data(), devices.size(), cudaGLDeviceListAll))
        {
            auto vendor = reinterpret_cast<const char*>(glGetString(GL_VENDOR));
            auto renderer = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
            ROSFMT_INFO("Could not list CUDA devices: {} ({}). Your OpenGL renderer is {}/{}. Falling back to software decoding.\n",
                cudaGetErrorString(err), err, vendor, renderer
            );
            return;
        }

        if(numDevices == 0)
        {
            ROSFMT_INFO("No CUDA device matching your GLX context found. Falling back to software decoding.\n");
            return;
        }

        m_cudaDevice = devices[0];
        cudaSetDevice(m_cudaDevice);
    }();

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
    if(m_hwFramesCtx)
        av_buffer_unref(&m_hwFramesCtx);

    if(m_codecCtx)
        avcodec_free_context(&m_codecCtx);

    if(m_parserCtx)
    {
        av_parser_close(m_parserCtx);
        m_parserCtx = nullptr;
    }

    if(m_stream)
        cudaStreamDestroy(m_stream);

    if(m_context)
        glXDestroyContext(m_display, m_context);
    if(m_pbuffer)
        glXDestroyPbuffer(m_display, m_pbuffer);
    if(m_display)
        XCloseDisplay(m_display);
}

bool Decoder::Private::initializeDecoder(const std::string& encoding, int width, int height)
{
    if(m_codecCtx)
        return true;

    if(!m_contextCurrent)
    {
        glXMakeContextCurrent(m_display, m_pbuffer, m_pbuffer, m_context);
        m_contextCurrent = true;
    }

    if(!m_stream)
    {
        // Init our own CUDA stream
        if(auto err = cudaSetDevice(m_cudaDevice))
        {
            fprintf(stderr, "Could not set CUDA device: %s (%d)\n", cudaGetErrorString(err), err);
            std::abort();
        }

        if(auto err = cuCtxGetCurrent(&m_cudaCtx))
            throw std::runtime_error{fmt::format("Could not get current CUDA context: {}", err)};

        if(auto err = cudaStreamCreateWithFlags(&m_stream, cudaStreamDefault))
            throw std::runtime_error{fmt::format("Could not create CUDA stream (error {})", err)};

        int device;
        cudaGetDevice(&device);

        cudaDeviceProp props{};

        cudaGetDeviceProperties(&props, device);

        m_nppCtx.hStream = m_stream;
        m_nppCtx.nCudaDeviceId = device;
        m_nppCtx.nMultiProcessorCount = props.multiProcessorCount;
        m_nppCtx.nMaxThreadsPerMultiProcessor = props.maxThreadsPerMultiProcessor;
        m_nppCtx.nMaxThreadsPerBlock = props.maxThreadsPerBlock;
        m_nppCtx.nSharedMemPerBlock = props.sharedMemPerBlock;

        cudaDeviceGetAttribute(&m_nppCtx.nCudaDevAttrComputeCapabilityMajor, cudaDevAttrComputeCapabilityMajor, device);
        cudaDeviceGetAttribute(&m_nppCtx.nCudaDevAttrComputeCapabilityMinor, cudaDevAttrComputeCapabilityMinor, device);
        cudaStreamGetFlags(m_stream, &m_nppCtx.nStreamFlags);
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
    else if(strcasecmp(encoding.c_str(), "h265") == 0)
        codecID = AV_CODEC_ID_H265;
    else if(strcasecmp(encoding.c_str(), "bgr8; jpeg compressed bgr8") == 0)
        codecID = AV_CODEC_ID_MJPEG;
    else if(strcasecmp(encoding.c_str(), "rgb8; jpeg compressed bgr8") == 0)
        codecID = AV_CODEC_ID_MJPEG;
    else if(strcasecmp(encoding.c_str(), "rgb8; jpeg compressed rgb8") == 0)
        codecID = AV_CODEC_ID_MJPEG;
    else if(strcasecmp(encoding.c_str(), "rgba8; jpeg compressed bgr8") == 0)
        codecID = AV_CODEC_ID_MJPEG;
    else if(strcasecmp(encoding.c_str(), "jpeg") == 0)
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

    AVBufferRef *hw_device_ctx = av_hwdevice_ctx_alloc(AV_HWDEVICE_TYPE_CUDA);
    AVHWDeviceContext* hw_device = reinterpret_cast<AVHWDeviceContext*>(hw_device_ctx->data);

    AVCUDADeviceContext* cuda_context = reinterpret_cast<AVCUDADeviceContext*>(hw_device->hwctx);
    cuda_context->cuda_ctx = m_cudaCtx;
    cuda_context->stream = m_stream;

    if(av_hwdevice_ctx_init(hw_device_ctx) < 0)
        throw std::runtime_error{"Could not initialize CUDA hwaccel for ffmpeg"};

    m_hwFramesCtx = av_hwframe_ctx_alloc(hw_device_ctx);

    m_codecCtx->hw_device_ctx = av_buffer_ref(hw_device_ctx);
    m_codecCtx->hw_frames_ctx = av_buffer_ref(m_hwFramesCtx);

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

    frameData->prepare(dataWidth, dataHeight, m_stream);
    frameData->texture.map();


    if(frame->format == AV_PIX_FMT_CUDA && (colorRange == AVCOL_RANGE_UNSPECIFIED || colorRange == AVCOL_RANGE_MPEG))
    {
        const uint8_t* pSrc[2] = {
            frame->data[0],
            frame->data[1]
        };

        NppiSize roi{dataWidth, dataHeight};

        if(auto err = nppiNV12ToRGB_709CSC_8u_P2C3R_Ctx(pSrc, frame->linesize[0], frameData->devBuffer, dataWidth * 3, roi, m_nppCtx))
            ROSFMT_ERROR("Could not convert NV12 to RGB: {}", err);

        const int dstOrder[4] = {0, 1, 2, 3};

        if(auto err = nppiSwapChannels_8u_C3C4R_Ctx(
                frameData->devBuffer, 3*frame->width,
                frameData->devBuffer2, 4*frame->width,
                roi, dstOrder, 0xff,
                m_nppCtx
            ))
        {
            ROSFMT_ERROR("Could not convert RGB to RGBA: {}", err);
        }

        if(auto err = cudaMemcpy2DToArrayAsync(
            frameData->texture.deviceArray(), 0, 0,
            frameData->devBuffer2, dataWidth*4, dataWidth*4, dataHeight,
            cudaMemcpyDeviceToDevice, m_stream))
        {
            ROSFMT_ERROR("Could not transfer to texture: {}", err);
        }
    }
    else if(frame->format == AV_PIX_FMT_CUDA && colorRange == AVCOL_RANGE_JPEG)
    {
        NppiSize roi{dataWidth, dataHeight};

        uint8_t* planes[3] = {
            frameData->devBuffer,
            frameData->devBuffer + dataWidth*dataHeight,
            frameData->devBuffer + 2*dataWidth*dataHeight,
        };
        int dstStep[3] = {
            dataWidth, dataWidth, dataWidth
        };

        if(auto err = nppiYCbCr420_8u_P2P3R_Ctx(frame->data[0], frame->linesize[0], frame->data[1], frame->linesize[1], planes, dstStep, roi, m_nppCtx))
            ROSFMT_ERROR("Could not convert JPEG NV12 to YUV420: {}", err);

        if(auto err = nppiYCbCr420ToRGB_8u_P3C3R_Ctx(planes, dstStep, frameData->devBuffer2, dataWidth * 3, roi, m_nppCtx))
        {
            ROSFMT_ERROR("Could not convert JPEG YUV420 to RGB: {}", err);
        }

        const int dstOrder[4] = {0, 1, 2, 3};

        if(auto err = nppiSwapChannels_8u_C3C4R_Ctx(
                frameData->devBuffer2, 3*frame->width,
                frameData->devBuffer, 4*frame->width,
                roi, dstOrder, 0xff,
                m_nppCtx
            ))
        {
            ROSFMT_ERROR("Could not convert RGB to RGBA: {}", err);
        }

        if(auto err = cudaMemcpy2DToArrayAsync(
            frameData->texture.deviceArray(), 0, 0,
            frameData->devBuffer, dataWidth*4, dataWidth*4, dataHeight,
            cudaMemcpyDeviceToDevice, m_stream))
        {
            ROSFMT_ERROR("Could not transfer to texture: {}", err);
        }
    }
    else if(frame->format == AV_PIX_FMT_BGR24)
    {
        NppiSize roi{dataWidth, dataHeight};

        const int dstOrder[4] = {2, 1, 0, 3};

        if(auto err = nppiSwapChannels_8u_C3C4R_Ctx(
                frame->data[0], frame->linesize[0],
                frameData->devBuffer, 4*frame->width,
                roi, dstOrder, 0xff,
                m_nppCtx
            ))
        {
            ROSFMT_ERROR("Could not convert BGR to RGBA: {}", err);
        }

        if(auto err = cudaMemcpy2DToArrayAsync(
            frameData->texture.deviceArray(), 0, 0,
            frameData->devBuffer, dataWidth*4, dataWidth*4, dataHeight,
            cudaMemcpyDeviceToDevice, m_stream))
        {
            ROSFMT_ERROR("Could not transfer to texture: {}", err);
        }
    }
    else if(frame->format == AV_PIX_FMT_RGB24)
    {
        NppiSize roi{dataWidth, dataHeight};

        const int dstOrder[4] = {0, 1, 2, 3};

        if(auto err = nppiSwapChannels_8u_C3C4R_Ctx(
                frame->data[0], frame->linesize[0],
                frameData->devBuffer, 4*frame->width,
                roi, dstOrder, 0xff,
                m_nppCtx
            ))
        {
            ROSFMT_ERROR("Could not convert RGB24 to RGBA: {}", err);
        }

        if(auto err = cudaMemcpy2DToArrayAsync(
            frameData->texture.deviceArray(), 0, 0,
            frameData->devBuffer, dataWidth*4, dataWidth*4, dataHeight,
            cudaMemcpyDeviceToDevice, m_stream))
        {
            ROSFMT_ERROR("Could not transfer to texture: {}", err);
        }
    }
    else if(frame->format == AV_PIX_FMT_GRAY8)
    {
        NppiSize roi{dataWidth, dataHeight};

        if(auto err = nppiDup_8u_C1AC4R_Ctx(
                frame->data[0], frame->linesize[0],
                frameData->devBuffer, 4*frame->width,
                roi,
                m_nppCtx
            ))
        {
            ROSFMT_ERROR("Could not convert gray to RGBA: {}", err);
        }

        if(auto err = cudaMemcpy2DToArrayAsync(
            frameData->texture.deviceArray(), 0, 0,
            frameData->devBuffer, dataWidth*4, dataWidth*4, dataHeight,
            cudaMemcpyDeviceToDevice, m_stream))
        {
            ROSFMT_ERROR("Could not transfer to texture: {}", err);
        }
    }
    else if(frame->format == AV_PIX_FMT_RGBA)
    {
        if(auto err = cudaMemcpy2DToArrayAsync(
            frameData->texture.deviceArray(), 0, 0,
            frame->data[0], dataWidth*4, dataWidth*4, dataHeight,
            cudaMemcpyDeviceToDevice, m_stream))
        {
            ROSFMT_ERROR("Could not transfer to texture: {}", err);
        }
    }
    else
    {
        ROSFMT_ERROR("Unsupported pixel format: {}", safePixelFormatName(static_cast<AVPixelFormat>(frame->format)));
    }


    frameData->texture.unmap();

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
    else if(msg->encoding == "rgba8")
        format = AV_PIX_FMT_RGBA;
    else
    {
        ROSFMT_WARN_THROTTLE(1.0, "Received image with unsupported encoding '{}', ignoring", msg->encoding);
        return false;
    }

    AVFrame* frame = av_frame_alloc();

    if(cudaMalloc(&frame->data[0], msg->data.size()) != 0)
    {
        ROSFMT_ERROR("Out of GPU memory");
        return false;
    }

    cudaMemcpyAsync(frame->data[0], msg->data.data(), msg->data.size(), cudaMemcpyHostToDevice, m_stream);

    frame->width = msg->width;
    frame->height = msg->height;
    frame->linesize[0] = msg->step;
    frame->format = format;

    pushFrame(frame);

    if(auto error = cudaFree(frame->data[0]))
        ROSFMT_ERROR("cudaFree failed: {}", cudaGetErrorString(error));

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

void Decoder::Private::shutdown()
{
    {
        std::unique_lock<std::mutex> lock{m_freeOutMutex};
        m_shutdown = true;
    }
    m_freeOutCond.notify_all();
}

std::optional<Decoder::OutputFrame> Decoder::Private::getNewFrame()
{
    std::unique_lock<std::mutex> lock(m_jobMutex);

    if(m_outputQueue.empty())
        return {};

    auto& task = m_outputQueue.front();

    // Task is done, but the final GL transfer from buffer into texture might
    // still be running.

    GLenum syncStatus = glClientWaitSync(task.frameData->fence, 0, 0);
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

std::optional<Decoder::OutputFrame> Decoder::getNewFrame()
{
    return m_d->getNewFrame();
}

void Decoder::flush()
{
    m_d->flush();
}

void Decoder::shutdown()
{
    m_d->shutdown();
}

}

#endif
