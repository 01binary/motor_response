/*
  encoder.h

  Absolute encoder class
  Created 09/20/2023

  Copyright (C) 2023 Valeriy Novytskyy
  This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <str1ker/Adc.h>
#include "histogramFilter.h"
#include "utilities.h"

/*----------------------------------------------------------*\
| encoder class
\*----------------------------------------------------------*/

class encoder
{
private:
  // Queue size for subscribers and publishers
  const int QUEUE_SIZE = 16;

  // Defaults for analog input range
  const int ANALOG_MIN = 0;
  const int ANALOG_MAX = 1023;

  // Defaults for analog input filtering
  const int DEFAULT_THRESHOLD = 8;
  const int DEFAULT_AVERAGE = 16;

private:
  //
  // Configuration
  //

  // Input topic for listening to analog readings from absolute encoder
  std::string m_topic = "robot/adc";

  // Analog input channel
  int m_input;

  // Analog reading min
  int m_minReading = ANALOG_MIN;

  // Analog reading max
  int m_maxReading = ANALOG_MAX;

  // Position (joint state) min
  double m_minPos = 0.0;

  // Position (joint state) max
  double m_maxPos = 1.0;

  //
  // State
  //

  // Last filtered analog reading
  int m_reading = -1;

  // Last position mapped from last reading
  double m_position = std::numeric_limits<double>::infinity();

  // Filter for analog input
  histogramFilter m_filter;

  // Subscriber for receiving analog readings from motor absolute encoder
  ros::Subscriber m_adcSub;

public:
  //
  // Constructor
  //

  encoder(): m_filter(DEFAULT_THRESHOLD, DEFAULT_AVERAGE)
  {
  }

  encoder(
    const std::string& topic, int input, int minReading, int maxReading, double minPos, double maxPos, int filterThreshold, int filterAverage)
    : m_topic(topic)
    , m_filter(filterThreshold, filterAverage)
    , m_input(input)
    , m_minReading(minReading)
    , m_maxReading(maxReading)
    , m_minPos(minPos)
    , m_maxPos(maxPos)
  {
  }

public:
  int getReading() const
  {
    // Get current filtered analog reading
    return m_reading;
  }

  double getPosition() const
  {
    // Get current position mapped from filtered analog reading
    return m_position;
  }

  bool isReady() const
  {
    // Check if we have readings from the encoder
    if (m_position == std::numeric_limits<double>::infinity())
      return false;

    // Check if the buffer is filled with readings within threshold of each other
    if (!m_filter.isStable())
      return false;

    return true;
  }

  void configure()
  {
    // Read configuration settings
    ros::param::get("inputTopic", m_topic);
    ros::param::get("input", m_input);
    ros::param::get("minReading", m_minReading);
    ros::param::get("maxReading", m_maxReading);
    ros::param::get("minPos", m_minPos);
    ros::param::get("maxPos", m_maxPos);

    int threshold = DEFAULT_THRESHOLD, average = DEFAULT_AVERAGE;
    ros::param::get("threshold", threshold);
    ros::param::get("average", average);

    m_filter = histogramFilter(threshold, average);
  }

  void initialize(ros::NodeHandle& node)
  {
    // Subscribe to analog readings
    m_adcSub = node.subscribe<str1ker::Adc>(
      m_topic, QUEUE_SIZE, &encoder::feedback, this);
  }

  void feedback(const str1ker::Adc::ConstPtr& msg)
  {
    // Read analog input
    m_reading = m_filter(msg->adc[m_input]);

    // Re-map to position
    m_position = mapZero(
      (double)m_reading, (double)m_minReading, (double)m_maxReading, m_minPos, m_maxPos);
  }
};
