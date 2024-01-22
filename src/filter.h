/*

  ██ ██  █████ █████ █████ █████         █  █████  █████   ████ █████ █████ ████    ████ █████
 █  █  █ █   █   █   █   █ █    █        █  █    █ █      █     █   █ █   █ █   █  █     █
 █  █  █ █   █   █   █   █ █████        █   █████  ████    ██   █████ █   █ █   █   ██   ████
 █  █  █ █   █   █   █   █ █    █   ████    █    █ █         █  █     █   █ █   █     █  █
 █  █  █ █████   █   █████ █    █  █        █    █ █████ ████   █     █████ █   █ ████   █████
                                  █
  filter.h

  Simple histogram and moving average filter for analog input
  Created 09/20/2023

  Copyright (C) 2023 Valeriy Novytskyy
  This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <map>
#include <ros/ros.h>
#include <vector>

/*----------------------------------------------------------*\
| filter class
\*----------------------------------------------------------*/

class filter
{
private:
  // Sample with the smallest number of occurrences
  int m_min;

  // Smallest number of occurences
  int m_minCount;

  // Sample with the largest number of occurrences
  int m_max;

  // Largest number of occurrences
  int m_maxCount;

  // Bins of sample values and the number of occurences of each
  std::map<int, int> m_bins;

  // Difference between incoming value and last value that resets bins
  int m_threshold;

  // How many samples to average on the way out for additional smoothing
  int m_average;

  // Buffer for computing a moving average of the sample with highest count
  std::vector<int> m_buffer;

  // The next circular buffer index to fill in
  int m_next;

public:
  //
  // Constructor
  //

  filter(int threshold, int average)
      : m_threshold(threshold)
      , m_average(average)
      , m_min(-1)
      , m_minCount(-1)
      , m_max(-1)
      , m_maxCount(-1)
      , m_buffer(average)
      , m_next(0)
  {
  }

  //
  // Sampling
  //

  int operator() (int sample)
  {
    m_bins[sample]++;

    bool overMin = m_min != -1 && m_min - sample >= m_threshold;
    bool overMax = m_max != -1 && sample - m_max >= m_threshold;

    if (overMin || overMax)
    {
      // Reset histogram bins when the input changed significantly
      m_bins.clear();
      m_min = -1;
      m_minCount = -1;
      m_max = -1;
      m_maxCount = -1;

      return average(sample);
    }
    else
    {
      // Classify into bins, and return average of bin with most samples
      for (auto &bin : m_bins)
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

  bool isStable() const
  {
    int last = m_buffer[0];

    for (auto sample : m_buffer)
    {
      int diff = abs(sample - last);

      if (diff > m_threshold) return false;
    }

    return true;
  }

  void debug() const
  {
    std::string values;
    std::string separators;
    std::string counts;

    for (auto &bin : m_bins)
    {
      values += std::to_string(bin.first) + "\t|\t";
      separators += "----------------";
      counts += std::to_string(bin.second) + "\t|\t";
    }

    ROS_INFO("%s", values.c_str());
    ROS_INFO("%s", separators.c_str());
    ROS_INFO("%s", counts.c_str());
  }

private:
  int average(int output)
  {
    m_buffer[m_next++] = output;

    if (m_next == m_average)
    {
      m_next = 0;
    }

    int sum = 0;

    for (auto &value : m_buffer)
    {
      sum += value;
    }

    return int((double)sum / (double)m_average);
  }
};
