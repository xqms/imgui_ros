// Circular buffer for plotting
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef IMGUI_ROS_SCROLLING_BUFFER_H
#define IMGUI_ROS_SCROLLING_BUFFER_H

#include <vector>
#include <array>
#include <algorithm>

namespace imgui_ros
{

template<int BUFFER_SIZE>
class ScrollingBuffer
{
public:
    ScrollingBuffer() {}
    explicit ScrollingBuffer(int rows) { reset(rows); }

    const float* timeData() const
    { return m_time.data(); }

    const float* rowData(unsigned int row) const
    { return m_storage.data() + row*BUFFER_SIZE; }

    const float* rowAccData(unsigned int row) const
    { return m_storageAcc.data() + row*BUFFER_SIZE; }

    std::size_t rows() const
    { return m_rows; }

    std::size_t size() const
    { return m_size; }

    //! Time offset where we should start to read (oldest entry)
    std::size_t offset() const
    { return m_offset; }

    float maximumAcc() const
    { return m_maxAcc; }

    float maximumAcc(int row) const
    { return m_rowAccMax[row]; }

    float maximum() const
    { return m_max; }

    float maximum(int row) const
    { return m_rowMax[row]; }

    float lastTime() const
    {
        if(m_size != BUFFER_SIZE)
            return m_time[m_size-1];
        else
            return m_time[(m_offset + BUFFER_SIZE - 1) % BUFFER_SIZE];
    }

    float firstTime() const
    {
        if(m_size != BUFFER_SIZE)
            return m_time[0];
        else
            return m_time[m_offset % BUFFER_SIZE];
    }

    void reset(std::size_t rows);
    void addRow(unsigned int row);
    void push_back(float time, float* data);
private:
    std::array<float, BUFFER_SIZE> m_time;
    std::vector<float> m_storage;
    std::vector<float> m_storageAcc;

    std::vector<float> m_rowMax;
    std::vector<float> m_rowAccMax;

    std::size_t m_size = 0;
    std::size_t m_offset = 0;
    std::size_t m_rows = 0;

    float m_max = 0.0f;
    float m_maxAcc = 0.0f;
};

// Implementation
template<int BUFFER_SIZE>
void ScrollingBuffer<BUFFER_SIZE>::reset(std::size_t rows)
{
    m_storage.clear();
    m_storage.resize(rows * BUFFER_SIZE, 0.0f);
    m_storageAcc.clear();
    m_storageAcc.resize(rows * BUFFER_SIZE, 0.0f);
    m_rowMax.clear();
    m_rowMax.resize(rows, 0.0f);
    m_rowAccMax.clear();
    m_rowAccMax.resize(rows, 0.0f);
    m_rows = rows;
    m_size = 0;
    m_offset = 0;
    m_max = 0.0f;
}

template<int BUFFER_SIZE>
void ScrollingBuffer<BUFFER_SIZE>::addRow(unsigned int row)
{
    // Add new row in storage with zeros
    m_storage.insert(m_storage.begin() + row*BUFFER_SIZE, BUFFER_SIZE, 0.0f);
    m_rowMax.insert(m_rowMax.begin() + row, 1, 0.0f);

    // Add new row in storageAcc
    m_storageAcc.insert(m_storageAcc.begin() + row*BUFFER_SIZE, BUFFER_SIZE, 0.0f);
    m_rowAccMax.insert(m_rowAccMax.begin() + row, 1, 0.0f);

    // And initialize from row before
    if(row != 0)
    {
        std::copy(
            m_storageAcc.begin() + (row-1)*BUFFER_SIZE,
            m_storageAcc.begin() + row*BUFFER_SIZE,
            m_storageAcc.begin() + row*BUFFER_SIZE
        );
        m_rowAccMax[row] = m_rowAccMax[row-1];
    }

    m_rows++;
}

template<int BUFFER_SIZE>
void ScrollingBuffer<BUFFER_SIZE>::push_back(float time, float* data)
{
    // Attention: m_rows may be zero. In that case, data may be nullptr.

    if(m_size < BUFFER_SIZE)
    {
        float accum = 0.0f;
        for(std::size_t row = 0; row < m_rows; ++row)
        {
            m_storage[row * BUFFER_SIZE + m_size] = data[row];

            accum += data[row];
            m_storageAcc[row * BUFFER_SIZE + m_size] = accum;
        }
        m_time[m_size] = time;

        m_size++;
    }
    else
    {
        float accum = 0.0f;
        for(std::size_t row = 0; row < m_rows; ++row)
        {
            m_storage[row * BUFFER_SIZE + m_offset] = data[row];

            accum += data[row];
            m_storageAcc[row * BUFFER_SIZE + m_offset] = accum;
        }
        m_time[m_offset] = time;

        m_offset = (m_offset + 1) % BUFFER_SIZE;
    }

    // Update maximum
    if(m_rows != 0)
    {
        std::fill(m_rowMax.begin(), m_rowMax.end(), 0.0f);

        m_max = 0.0f;
        for(std::size_t row = 0; row < m_rows; ++row)
        {
            for(std::size_t time = 0; time < BUFFER_SIZE; ++time)
            {
                m_rowMax[row] = std::max(m_rowMax[row], m_storage[row*BUFFER_SIZE + time]);
            }
        }

        m_rowAccMax[0] = m_rowMax[0];
        for(std::size_t row = 1; row < m_rows; ++row)
            m_rowAccMax[row] = m_rowAccMax[row-1] + m_rowMax[row];

        m_max = *std::max_element(m_rowMax.begin(), m_rowMax.end());
        m_maxAcc = m_rowAccMax[m_rows-1];
    }
}

}

#endif
