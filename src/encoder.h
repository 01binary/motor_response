/*
                            ██
  ██ ██  ████ ███ ████ ████   █     █ ████  ████ ███ ████ ████ ███  ███ ████
 █  █  █ █  █  █  █  █ ███    █    █  ███   ███   ██ ████ █  █ █  █  ██ ███
 █  █  █ ████  █  ████ █  ██   █  █   █  ██ ████ ███ █    ████ █  █ ███ ████
                                ██
  encoder.h

  Absolute encoder class
  Created 09/20/2023

  Copyright (C) 2023 Valeriy Novytskyy
  This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <motor_response/Adc.h>
#include "filter.h"
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
  std::string m_topic = "adc";

  // Absolute input channel
  int m_absolute;

  // Relative input channel
  int m_relative;

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

  bool m_ready = false;

  // Last filtered absolute reading
  int m_reading = -1;

  // Last relative count
  int m_count = 0;

  // Last position
  double m_position = std::numeric_limits<double>::infinity();

  // Filter for absolute position readings
  filter m_filter;

  // Subscriber for receiving encoder readings
  ros::Subscriber m_sub;

public:
  //
  // Constructor
  //

  encoder(): m_filter(DEFAULT_THRESHOLD, DEFAULT_AVERAGE)
  {
  }

  encoder(
    const std::string& topic,
    int absoluteInput,
    int relativeInput,
    int minReading,
    int maxReading,
    double minPos,
    double maxPos,
    int filterThreshold,
    int filterAverage)
    : m_topic(topic)
    , m_filter(filterThreshold, filterAverage)
    , m_absolute(absoluteInput)
    , m_relative(relativeInput)
    , m_minReading(minReading)
    , m_maxReading(maxReading)
    , m_minPos(minPos)
    , m_maxPos(maxPos)
    , m_ready(false)
  {
  }

public:
  // Get current filtered absolute encoder reading
  inline int getReading() const
  {
    return m_reading;
  }

  // Get current relative encoder count
  inline int getCount() const
  {
    return m_count;
  }

  // Get current position mapped from filtered analog reading
  inline double getPosition() const
  {
    return m_position;
  }

  // Get minimum position
  inline double getMin() const
  {
    return m_minPos;
  }

  // Get maximum position
  inline double getMax() const
  {
    return m_maxPos;
  }

  // Determine if the encoder is ready to provide readings
  bool isReady() const
  {
    return m_ready;
  }

  void configure()
  {
    // Read configuration settings
    ros::param::get("inputTopic", m_topic);
    ros::param::get("absolute", m_absolute);
    if (!ros::param::get("relative", m_relative)) m_relative = -1;
    ros::param::get("minReading", m_minReading);
    ros::param::get("maxReading", m_maxReading);
    ros::param::get("minPos", m_minPos);
    ros::param::get("maxPos", m_maxPos);

    int threshold = DEFAULT_THRESHOLD, average = DEFAULT_AVERAGE;
    ros::param::get("threshold", threshold);
    ros::param::get("average", average);

    m_filter = filter(threshold, average);
  }

  void initialize(ros::NodeHandle& node)
  {
    // Subscribe to analog readings
    m_sub = node.subscribe<motor_response::Adc>(
      m_topic, QUEUE_SIZE, &encoder::feedback, this);
  }

  void feedback(const motor_response::Adc::ConstPtr& msg)
  {
    // Read absolute encoder
    m_reading = m_filter(msg->adc[m_absolute]);

    // Read relative encoder
    if (m_relative != -1) m_count = msg->adc[m_relative];

    // Re-map to position
    m_position = map(
      (double)m_reading,
      (double)m_minReading,
      (double)m_maxReading,
      m_minPos,
      m_maxPos);

    // Check if the position is ready to be used
    if (!m_ready)
    {
      m_ready = m_filter.isStable();
    }
  }
};
