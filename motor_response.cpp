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

#include <ros/ros.h>
#include <str1ker/Adc.h>
#include <str1ker/Pwm.h>
#include "histogramFilter.h"

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

int spinRate;
int input;
int lpwm;
int rpwm;
int minPwm = PWM_MIN, maxPwm = PWM_MAX;
double minVelocity = 0.0, maxVelocity = 1.0;
int minReading = ANALOG_MIN, maxReading = ANALOG_MAX;
double minPos = 0.0, maxPos = 1.0;
histogramFilter filter(8, 8);

// State

double pos = 0.0;

// ROS interface
ros::Subscriber sub;
ros::Publisher pub;

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

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

    // Run node
    ros::Rate rate(spinRate);

    while(node.ok())
    {
        ROS_INFO("position %.2g", pos);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
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
  int reading = filter(msg->adc[input]);

  // Re-map to position
  pos = map((double)reading, (double)minReading, (double)maxReading, minPos, maxPos);
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
