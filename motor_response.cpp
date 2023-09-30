/*
  motor_response.cpp

  DC motor characterization tool
  Created 09/20/2023

  Copyright (C) 2023 Valeriy Novytskyy
  This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <map>
#include <memory>
#include <limits>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <str1ker/Adc.h>
#include <str1ker/Pwm.h>

#include "histogramFilter.h"

/*----------------------------------------------------------*\
| Types
\*----------------------------------------------------------*/

struct step
{
  double velocity;
  double duration;
};

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const int QUEUE_SIZE = 16;

const int PWM_MIN = 0;
const int PWM_MAX = 255;

const int ANALOG_MIN = 0;
const int ANALOG_MAX = 1024;

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

// Configuration
std::string inputTopic;
std::string outputTopic;
std::string outputLog;
int spinRate;
int input;
int lpwm;
int rpwm;
int minPwm = PWM_MIN, maxPwm = PWM_MAX;
double minVelocity = 0.0, maxVelocity = 1.0;
int minReading = ANALOG_MIN, maxReading = ANALOG_MAX;
double minPos = 0.0, maxPos = 1.0;
int filterThreshold;
int filterAverage;
std::unique_ptr<histogramFilter> pFilter;
std::vector<step> steps;

// State
std::ofstream logFile;
double position = 0.0;
size_t currentStep = 0;
bool running = false;

// ROS interface
ros::Subscriber sub;
ros::Publisher pub;

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

void configure();
void initialize(ros::NodeHandle node);
void command(double velocity);
void feedback(const str1ker::Adc::ConstPtr& msg);
template<class T> T map(T value, T min, T max, T targetMin, T targetMax);
template<class T> T clamp(T value, T min, T max);

/*----------------------------------------------------------*\
| Package entry point
\*----------------------------------------------------------*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_response");
  ros::NodeHandle node;

  // Configure node
  configure();

  // Initialize node
  initialize(node);

  // Run node
  ros::Rate rate(spinRate);
  ros::Time logStart = ros::Time::now();
  ros::Time start = ros::Time::now();

  while(node.ok())
  {
    if (running)
    {
      ros::Time time = ros::Time::now();
      ros::Duration duration = time - start;

      if (duration.toSec() >= steps[currentStep].duration)
      {
        currentStep++;
        start = time;

        if (currentStep >= steps.size())
        {
          break;
        }

        // Execute command
        command(steps[currentStep].velocity);
      }

      // Log command and position
      logFile << (time - logStart).toSec() << ", "
              << steps[currentStep].velocity << ", "
              << position << std::endl;

      ROS_INFO(
        "step %d\tvelocity %g\tposition %g",
        int(currentStep + 1), steps[currentStep].velocity, position
      );
    }
    else
    {
      ROS_INFO("position %g", position);
    }

    ros::spinOnce();
    rate.sleep();
  }

  logFile.close();

  return 0;
}

/*----------------------------------------------------------*\
| Configuration
\*----------------------------------------------------------*/

void configure()
{
  ros::param::get("rate", spinRate);
  ros::param::get("inputTopic", inputTopic);
  ros::param::get("outputTopic", outputTopic);
  ros::param::get("input", input);
  ros::param::get("lpwm", lpwm);
  ros::param::get("rpwm", rpwm);
  ros::param::get("minPwm", minPwm);
  ros::param::get("maxPwm", maxPwm);
  ros::param::get("minVelocity", minVelocity);
  ros::param::get("maxVelocity", maxVelocity);
  ros::param::get("minReading", minReading);
  ros::param::get("maxReading", maxReading);
  ros::param::get("minPos", minPos);
  ros::param::get("maxPos", maxPos);
  ros::param::get("threshold", filterThreshold);
  ros::param::get("average", filterAverage);
  ros::param::get("log", outputLog);

  int stepId = 1;
  std::map<std::string, double> stepParams;

  while (ros::param::get(std::string("steps/step") + std::to_string(stepId++), stepParams))
  {
    step s;
    s.velocity = stepParams["velocity"];
    s.duration = stepParams["duration"];
    steps.push_back(s);
  }

  if (steps.size() > 0)
  {
    running = true;
  }
}

/*----------------------------------------------------------*\
| Initialization
\*----------------------------------------------------------*/

void initialize(ros::NodeHandle node)
{
  // Initialize analog subscriber
  sub = node.subscribe<str1ker::Adc>(
    inputTopic,
    QUEUE_SIZE,
    &feedback
  );

  // Initialize analog publisher
  pub = node.advertise<str1ker::Pwm>(
    outputTopic,
    QUEUE_SIZE
  );

  // Initialize input filter
  pFilter = std::unique_ptr<histogramFilter>(
    new histogramFilter(filterThreshold, filterAverage));

  // Open the log file
  logFile.open(outputLog);
}

/*----------------------------------------------------------*\
| Command
\*----------------------------------------------------------*/

void command(double velocity)
{
  double commandedVelocity = clamp(abs(velocity), minVelocity, maxVelocity);

  uint8_t dutyCycle = (uint8_t)map(
    commandedVelocity, minVelocity, maxVelocity, (double)minPwm, (double)maxPwm);

  str1ker::Pwm msg;
  msg.channels.resize(2);

  // RPWM
  msg.channels[0].channel = lpwm;
  msg.channels[0].mode = str1ker::PwmChannel::MODE_ANALOG;
  msg.channels[0].value = (velocity >= 0 ? dutyCycle : 0);
  msg.channels[0].duration = 0;

  // LPWM
  msg.channels[1].channel = rpwm;
  msg.channels[1].mode = str1ker::PwmChannel::MODE_ANALOG;
  msg.channels[1].value = (velocity >= 0 ? 0 : dutyCycle);
  msg.channels[1].duration = 0;

  pub.publish(msg);
}

/*----------------------------------------------------------*\
| Feedback
\*----------------------------------------------------------*/

void feedback(const str1ker::Adc::ConstPtr& msg)
{
  // Read analog input
  int reading = (*pFilter)(msg->adc[input]);

  // Re-map to position
  position = map((double)reading, (double)minReading, (double)maxReading, minPos, maxPos);
}

/*----------------------------------------------------------*\
| Utilities
\*----------------------------------------------------------*/

template<class T> T clamp(T value, T min, T max)
{
  if (value < min) return min;
  if (value > max) return max;

  return value;
}

template<class T> T map(T value, T min, T max, T targetMin, T targetMax)
{
  return (value - min) / (max - min) * (targetMax - targetMin) + targetMin;
}
