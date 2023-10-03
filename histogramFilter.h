/*
  histogramFilter.h

  Simple histogram and moving average filter for analog input
  Created 09/20/2023

  Copyright (C) 2023 Valeriy Novytskyy
  This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <map>
#include <vector>
#include <ros/ros.h>

/*----------------------------------------------------------*\
| histogramFilter class
\*----------------------------------------------------------*/

class histogramFilter
{
private:
    // Threshold is the minimum difference between incoming value and last value to trigger a reset
    int m_threshold;
    // Average is how many samples to average on the way out (some additional smoothing)
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
            // Reset histogram bins when the input is changing significantly
            m_bins.clear();
            m_min = -1;
            m_minCount = -1;
            m_max = -1;
            m_maxCount = -1;
            return average(sample);
        }
        else
        {
            // Classify input into bins, and return the average of the bin with the most samples
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
