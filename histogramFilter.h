#include <map>
#include <vector>
#include <ros/ros.h>

class histogramFilter
{
private:
    int m_threshold;
    int m_average;
    int m_min;
    int m_minCount;
    int m_max;
    int m_maxCount;
    std::map<int, int> m_bins;
    std::vector<int> m_buffer;
    int m_next;

public:
    histogramFilter(int threshold, int average):
        m_threshold(threshold),
        m_average(average),
        m_min(-1),
        m_minCount(-1),
        m_max(-1),
        m_maxCount(-1),
        m_buffer(average)
    {
    }

    int operator () (int sample)
    {
        m_bins[sample]++;

        bool breachedMin = m_min != -1 && m_min - sample >= m_threshold;
        bool breachedMax = m_max != -1 && sample - m_max >= m_threshold;

        if (breachedMin || breachedMax)
        {
            m_bins.clear();
            m_min = -1;
            m_minCount = -1;
            m_max = -1;
            m_maxCount = -1;
            return average(sample);
        }
        else
        {
            for (auto& bin : m_bins)
            {
                if (bin.second > m_maxCount || m_maxCount == -1)
                {
                    m_maxCount = bin.second;
                    m_max = bin.first;
                }

                if (bin.second < m_minCount || m_minCount == -1)
                {
                    m_minCount = bin.second;
                    m_min = bin.first;
                }
            }

            return average(m_max);
        }
    }

    int average(int output)
    {
        m_buffer[m_next++] = output;
        
        if (m_next == m_average)
        {
            m_next = 0;
        }

        int sum = 0;

        for (auto& value : m_buffer)
        {
            sum += value;
        }

        return int((double)sum / (double)m_average);
    }

    void debug()
    {
        std::string values;
        std::string separators;
        std::string counts;

        for (auto& bin : m_bins)
        {
            values += std::to_string(bin.first) + "\t|\t";
            separators += "----------------";
            counts += std::to_string(bin.second) + "\t|\t";
        }

        ROS_INFO(values.c_str());
        ROS_INFO(separators.c_str());
        ROS_INFO(counts.c_str());
    }
};
