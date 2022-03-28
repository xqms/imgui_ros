// Provides abstraction of the outer GUI framework
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef IMGUI_ROS_CONTEXT_H
#define IMGUI_ROS_CONTEXT_H

#include <ros/subscriber.h>
#include <ros/transport_hints.h>
#include <topic_tools/shape_shifter.h>

namespace imgui_ros
{

class Subscriber
{
public:
    class Impl
    {
    public:
        virtual int getNumPublishers() = 0;
        virtual void shutdown() = 0;
    };

    Subscriber()
    {}

    explicit Subscriber(const std::shared_ptr<Impl>& impl)
     : m_impl{impl}
    {}

    void shutdown() { m_impl->shutdown(); }
    int getNumPublishers() { return m_impl->getNumPublishers(); }

private:
    std::shared_ptr<Impl> m_impl;
};

class Context
{
public:
    virtual void setWindowTitle(const std::string& title) = 0;
    virtual void setUpdateRate(float updateRate) = 0;

    virtual ros::NodeHandle& nodeHandle() = 0;

    template<class Msg, typename Cb>
    Subscriber subscribe(const std::string& topic, int queue, const Cb& cb, const ros::TransportHints& hints = {})
    {
        auto rawCb = [=](const boost::shared_ptr<const topic_tools::ShapeShifter>& shifter){
            cb(shifter->instantiate<Msg>());
        };

        return subscribeRaw(topic, queue, rawCb, hints);
    }

protected:
    using RawCb = std::function<void(const boost::shared_ptr<const topic_tools::ShapeShifter>&)>;
    virtual Subscriber subscribeRaw(const std::string& topic, int queue, const RawCb& rawCb, const ros::TransportHints& hints = {}) = 0;
};

}

#endif

