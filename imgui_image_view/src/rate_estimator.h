// Smooth rate estimator
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef RATE_ESTIMATOR_H
#define RATE_ESTIMATOR_H

#include <memory>
#include <ros/time.h>

namespace imgui_image_view
{

class RateEstimator
{
public:
    void put(const ros::Time& time);

    float rateAt(const ros::Time& time) const;
    inline float rateNow() const
    { return rateAt(ros::Time::now()); }

private:
    static constexpr float HALF_LIFE = 0.5f;
    static constexpr float LOG05 = -0.6931471805599453f; // std::log(0.5f)
    static constexpr float DECAY = -LOG05/HALF_LIFE;

    ros::Time m_lastMessageTime;
    float m_lambdaSmoothLast = 0.0f;
    float m_lambdaLast = 0.0f;
};

void RateEstimator::put(const ros::Time& time)
{
    if(time == ros::Time(0))
    {
        m_lambdaSmoothLast = 0.0f;
        m_lambdaLast = 0.0f;
        return;
    }
    else if(time < m_lastMessageTime)
    {
        m_lambdaSmoothLast = 0.0f;
        m_lambdaLast = 0.0f;
        m_lastMessageTime = time;
        return;
    }

    // The smooth rate estimate is taken from here:
    // https://stackoverflow.com/a/23617678
    float tDelta = (time - m_lastMessageTime).toSec();

    float expL = std::exp(-DECAY * tDelta);

    m_lambdaSmoothLast = DECAY * tDelta * expL * m_lambdaLast + expL * m_lambdaSmoothLast;
    m_lambdaLast = DECAY + expL * m_lambdaLast;
    m_lastMessageTime = time;
}

float RateEstimator::rateAt(const ros::Time& time) const
{
    float tDelta = (time - m_lastMessageTime).toSec();
    float expL = std::exp(-DECAY * tDelta);

    // Bias correction
    float t0Delta = (time - ros::Time(0)).toSec();
    float S = (1.0f + DECAY * t0Delta) * std::exp(-DECAY * t0Delta);

    return (DECAY * tDelta * expL * m_lambdaLast + expL * m_lambdaSmoothLast) / (1.0f - S);
}

}

#endif
