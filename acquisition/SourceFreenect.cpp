#include <iostream>
#include <list>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <libfreenect.h>
#include <opencv2/opencv.hpp>
#include "SourceFreenect.hpp"

namespace Freenect {

class Context
{
public:
    using Action = std::function<void()>;

private:
    ::freenect_context* ctx;
    std::atomic_bool is_running {false};
    std::thread t;
    Context();
    Context(const Context&) = delete;

    std::mutex m;
    using Lock = std::unique_lock<std::mutex>;
    std::list<std::function<void()>> actions;

public:
    ~Context();
    static Context& getInstance();
    static void log_cb(::freenect_context*, ::freenect_loglevel, const char *msg)
    {
        std::clog << "Freenect: " << msg << std::endl;
    }
    operator ::freenect_context* () const {
        return ctx;
    }
    void post(const Action& f) {
        Lock l(m);
        actions.push_back(f);
        l.unlock();
    }
};

Context::Context()
{
    if (::freenect_init(&ctx, nullptr) < 0)
        throw std::runtime_error("Error initializing freenect context");
    ::freenect_set_log_callback(ctx, &log_cb);
    ::freenect_set_log_level(ctx, ::FREENECT_LOG_NOTICE);
    t = std::thread([this] {
        timeval timeout {0, 500000};
        is_running = true;
        int count = 0;
        while (is_running) {
            const auto ret = ::freenect_process_events_timeout(ctx, &timeout);
            if (ret != 0)
                std::cerr << "Ret: " << ret << std::endl;
            for (;;) {
                Action f;
                Lock l(m);
                if (actions.empty() == false) {
                    f = std::move(actions.front());
                    actions.pop_front();
                }
                l.unlock();
                if (f == false)
                    break;
                f();
            };
            //std::clog << "Process events " << ++count << std::endl;
        }
    });
}

Context::~Context()
{
    is_running = false;
    t.join();
    ::freenect_shutdown(ctx);
}

Context& Context::getInstance()
{
    static Context context;
    return context;
}

class Device
{
    std::string camera_serial;
    ::freenect_device* dev;

    static const int n_values = 2048;
    unsigned short disparity_to_depth[n_values];

    using Lock = std::unique_lock<std::mutex>;
    std::mutex frame_mutex;
    std::condition_variable frame_cond;

    std::vector<unsigned short> depth_buffer_back, depth_buffer_mid, depth_buffer_front;
    bool got_depth = false;

    std::vector<char> video_buffer_back, video_buffer_mid, video_buffer_front;
    bool got_video = false;

    static void video_cb(freenect_device *dev, void *video, uint32_t timestamp);
    static void depth_cb(freenect_device *dev, void *depth, uint32_t timestamp);

    void video_callback(void *depth, uint32_t timestamp);
    void depth_callback(void *depth, uint32_t timestamp);

    bool is_video_running = false, is_depth_running = false;

public:
    Device(const int i);
    ~Device();
    std::string getSerial() const;
    void startVideo(freenect_resolution resolution, freenect_video_format format);
    void startDepth();
    void stopVideo();
    void stopDepth();
    void waitAll();
    const char* getVideo() const;
    const unsigned short* getDepth() const;
};

void Device::video_cb(freenect_device* dev, void* video, uint32_t timestamp)
{
    Device* p = (Device*)::freenect_get_user(dev);
    p->video_callback(video, timestamp);
}

void Device::depth_cb(freenect_device* dev, void* depth, uint32_t timestamp)
{
    Device* p = (Device*)::freenect_get_user(dev);
    p->depth_callback(depth, timestamp);
}

void Device::video_callback(void* video, uint32_t timestamp)
{
    const char* v = (const char*)video;
    const auto mode = ::freenect_get_current_video_mode(dev);
    video_buffer_back.resize(mode.bytes);
    std::copy(v, v + mode.bytes, video_buffer_back.begin());
    Lock l(frame_mutex);
    std::swap(video_buffer_back, video_buffer_mid);
    got_video = true;
    frame_cond.notify_all();
}

void Device::depth_callback(void* depth, uint32_t timestamp)
{
    unsigned short* d = (unsigned short*)depth;
    const auto mode = ::freenect_get_current_depth_mode(dev);
    const int n = mode.height * mode.width;
    depth_buffer_back.resize(n);
    for (int i = 0; i < n; ++i)
        depth_buffer_back[i] = disparity_to_depth[d[i]];
    Lock l(frame_mutex);
    std::swap(depth_buffer_back, depth_buffer_mid);
    got_depth = true;
    frame_cond.notify_all();
}

Device::Device(const int i)
{
    auto& ctx = Context::getInstance();
    ::freenect_device_attributes* attr;
    const auto n = ::freenect_list_device_attributes(ctx, &attr);
    if (n < i + 1) {
        ::freenect_free_device_attributes(attr);
        throw std::logic_error("Kinect device not found");
    }
    camera_serial = attr[i].camera_serial;
    ::freenect_free_device_attributes(attr);
    if (::freenect_open_device(ctx, &dev, i) < 0)
        throw std::runtime_error("Error opening the Kinect device");
    ::freenect_set_user(dev, this);
    ::freenect_set_video_callback(dev, video_cb);
    ::freenect_set_depth_callback(dev, depth_cb);
    const std::string calibration = camera_serial + ".yml";
    cv::FileStorage in(calibration, cv::FileStorage::READ);
    if (in.isOpened() == false)
        throw std::logic_error("Unable toopen calibration file " + calibration);
    double a, b;
    in["alpha"] >> a;
    in["beta"] >> b;

    for (int i = 0; i < n_values; ++i)
        disparity_to_depth[i] = static_cast<unsigned short>(1000.0 / (a * i + b));
}

Device::~Device()
{
    stopDepth();
    stopVideo();
    ::freenect_close_device(dev);
}

std::string Device::getSerial() const
{
    return camera_serial;
}

struct Data
{
    using Lock = std::unique_lock<std::mutex>;
    bool completed = false;
    std::mutex m;
    std::condition_variable c;
};

void Device::startVideo(::freenect_resolution resolution, ::freenect_video_format format)
{
    Data data;
    Context::Action f = [this, &data, resolution, format] {
        const auto mode = ::freenect_find_video_mode(resolution, format);
        ::freenect_set_video_mode(dev, mode);
        ::freenect_start_video(dev);
        Data::Lock l(data.m);
        data.completed = true;
        data.c.notify_all();
    };
    Context::getInstance().post(f);
    Data::Lock l(data.m);
    while (data.completed == false)
        data.c.wait(l);
    is_video_running = true;
}

void Device::startDepth()
{
    Data data;
    Context::Action f = [this, &data] {
        auto depth_mode = ::freenect_find_depth_mode(::FREENECT_RESOLUTION_MEDIUM, ::FREENECT_DEPTH_11BIT);
        ::freenect_set_depth_mode(dev, depth_mode);
        ::freenect_start_depth(dev);
        Data::Lock l(data.m);
        data.completed = true;
        data.c.notify_all();
    };
    Context::getInstance().post(f);
    Data::Lock l(data.m);
    while (data.completed == false)
        data.c.wait(l);
    is_depth_running = true;
}

void Device::stopVideo()
{
    Data data;
    Context::Action f = [this, &data] {
        ::freenect_stop_video(dev);
        Data::Lock l(data.m);
        data.completed = true;
        data.c.notify_all();
    };
    Context::getInstance().post(f);
    Data::Lock l(data.m);
    while (data.completed == false)
        data.c.wait(l);
    is_video_running = false;
}

void Device::stopDepth()
{
    Data data;
    Context::Action f = [this, &data] {
        ::freenect_stop_depth(dev);
        Data::Lock l(data.m);
        data.completed = true;
        data.c.notify_all();
    };
    Context::getInstance().post(f);
    Data::Lock l(data.m);
    while (data.completed == false)
        data.c.wait(l);
    is_depth_running = false;
}

void Device::waitAll()
{
    Lock l(frame_mutex);
    while ((is_video_running && got_video == false) || (is_depth_running && got_depth == false))
        frame_cond.wait(l);
    if (got_video) {
        std::swap(video_buffer_mid, video_buffer_front);
        got_video = false;
    }
    if (got_depth) {
        std::swap(depth_buffer_mid, depth_buffer_front);
        got_depth = false;
    }
}

const char* Device::getVideo() const
{
    if (video_buffer_front.empty())
        throw std::logic_error("The video buffer is invalid");
    return video_buffer_front.data();
}

const unsigned short* Device::getDepth() const
{
    if (depth_buffer_front.empty())
        throw std::logic_error("The depth buffer is invalid");
    return depth_buffer_front.data();
}

}

struct SourceFreenect::Impl
{
    Freenect::Device dev;
    Impl(int id) : dev(id) {}
};

SourceFreenect::SourceFreenect(const int id) :
    p(new SourceFreenect::Impl(id))
{

}

SourceFreenect::~SourceFreenect()
{

}

void SourceFreenect::grab()
{
    p->dev.waitAll();
}

void SourceFreenect::startImage()
{
    p->dev.startVideo(::FREENECT_RESOLUTION_MEDIUM, ::FREENECT_VIDEO_RGB);
}

void SourceFreenect::startDepth()
{
    p->dev.startDepth();
}

void SourceFreenect::startIr()
{
    p->dev.startVideo(::FREENECT_RESOLUTION_HIGH, ::FREENECT_VIDEO_IR_8BIT);
}

void SourceFreenect::stopAll()
{
    p->dev.stopVideo();
    p->dev.stopDepth();
}

void SourceFreenect::getImage(char* rgb)
{
    const char* buffer = p->dev.getVideo();
    std::copy(buffer, buffer + 3 * 640 * 480, rgb);
}

void SourceFreenect::getDepth(char* depth)
{
    const char* buffer = (const char*)p->dev.getDepth();
    std::copy(buffer, buffer + 2 * 640 * 480, depth);
}

void SourceFreenect::getIr(char* ir)
{
    const char* buffer = p->dev.getVideo();
    std::copy(buffer, buffer + 1 * 1280 * 1024, ir);
}

int SourceFreenect::width() const
{
    return 640;
}

int SourceFreenect::height() const
{
    return 480;
}

std::string SourceFreenect::getSerial() const
{
    return p->dev.getSerial();
}
