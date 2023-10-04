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

//
// Standard library
//

#include <map>
#include <memory>
#include <limits>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

//
// ROS
//

#include <ros/ros.h>

//
// Messages
//

#include <str1ker/Adc.h>
#include <str1ker/Pwm.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

//
// Helpers
//

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

// Queue size for subscribers and publishers
const int QUEUE_SIZE = 16;

// Defaults for PWM range
const int PWM_MIN = 0;
const int PWM_MAX = 4096;

// Defaults for analog input range
const int ANALOG_MIN = 0;
const int ANALOG_MAX = 1023;

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

//
// Configuration
//

// Input topic for listening to analog readings from absolute encoder
std::string inputTopic;

// Input topic for listening to trajectory commands
std::string controlTopic;

// The joint to watch for in trajectory commands
std::string joint;

// Output topic for publishing PWM commands
std::string outputTopic;

// Output log CSV file for recording commands and readings
std::string outputLog;

// Playback rate
int spinRate;

// Analog input channel
int input;

// PWM output channels
int lpwm;
int rpwm;

// PWM output range (mapped form velocity)
int minPwm = PWM_MIN, maxPwm = PWM_MAX;

// Velocity range
double minVelocity = 0.0, maxVelocity = 1.0;

// Analog reading range
int minReading = ANALOG_MIN, maxReading = ANALOG_MAX;

// Position range (mapped from analog reading)
double minPos = 0.0, maxPos = 1.0;

// Filter for analog input
int filterThreshold;
int filterAverage;
std::unique_ptr<histogramFilter> pFilter;

// Steps to play back (velocity, duration)
std::vector<step> steps;

//
// State
//

// Opened log file for recording commands and readings
std::ofstream logFile;

// Last filtered analog reading
int reading = -1;

// Last position mapped from last reading
double position = std::numeric_limits<double>::infinity();

// Last PWM commands mapped from velocity command
uint16_t lpwmCommand = 0;
uint16_t rpwmCommand = 0;

// Last velocity command
double velocity = 0.0;

// Current step being played back and when it started
size_t currentStep = 0;
ros::Time stepTime;

//
// ROS interface
//

// Subscriber for receiving analog readings from motor absolute encoder
ros::Subscriber adcSub;

// Publisher for sending PWM commands to motor driver
ros::Publisher pwmPub;

// Subscriber for receiving follow joint trajectory commands
ros::Subscriber controlSub;

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

void configure();
void initialize(ros::NodeHandle node);
void playback(ros::Time time);
void control(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg);
void command(double velocity);
void feedback(const str1ker::Adc::ConstPtr& msg);
void record(double elapsed);
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
  ros::Time startTime = ros::Time::now();
  bool waiting = true;

  while(node.ok())
  {
    if (position == std::numeric_limits<double>::infinity())
    {
      // Wait until we have readings from the encoder
      ROS_INFO("waiting for feedback...");
    }
    else if (waiting)
    {
      // Initialize playback
      waiting = false;
      startTime = ros::Time::now();
      stepTime = ros::Time::now();
      ROS_INFO("ready");
    }
    else
    {
      ros::Time time = ros::Time::now();

      // Play back steps from configuration
      playback(time);

      // Record current command and position
      record((time - startTime).toSec());
    }

    ros::spinOnce();
    rate.sleep();
  }

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
  ros::param::get("controlTopic", controlTopic);
  ros::param::get("joint", joint);
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
}

/*----------------------------------------------------------*\
| Initialization
\*----------------------------------------------------------*/

void initialize(ros::NodeHandle node)
{
  // Initialize analog input subscriber
  adcSub = node.subscribe<str1ker::Adc>(
    inputTopic,
    QUEUE_SIZE,
    &feedback
  );

  // Initialize joint trajectory subscriber
  if (controlTopic.size())
  {
    ROS_INFO("subscribing to %s", controlTopic.c_str());
    controlSub = node.subscribe<control_msgs::FollowJointTrajectoryActionGoal>(
      controlTopic,
      QUEUE_SIZE,
      &control
    );
  }

  // Initialize PWM output publisher
  pwmPub = node.advertise<str1ker::Pwm>(
    outputTopic,
    QUEUE_SIZE
  );

  // Initialize analog input filtering
  pFilter = std::unique_ptr<histogramFilter>(
    new histogramFilter(filterThreshold, filterAverage));

  // Open the log file
  if (outputLog.size())
  {
    logFile.open(outputLog);

    if (!logFile.is_open())
    {
      ROS_ERROR("Failed to open log file %s", outputLog.c_str());
      exit(1);
    }
    else
    {
      // Write header row
      logFile << "time (sec)" << ", "
              << "velocity" << ", "
              << "LPWM" << ", "
              << "RPWM" << ", "
              << "position" << ", "
              << "reading" << std::endl;
    }
  }
}

/*----------------------------------------------------------*\
| Playback
\*----------------------------------------------------------*/

void playback(ros::Time time)
{
  if (!steps.size()) return;

  ros::Duration stepDuration = time - stepTime;

  if (stepDuration.toSec() >= steps[currentStep].duration)
  {
    // Go to the next step
    currentStep = (currentStep + 1) % steps.size();
    stepTime = time;

    // Execute command
    velocity = steps[currentStep].velocity;
    command(velocity);

    if (outputLog.size()) logFile.flush();
  }
}

/*----------------------------------------------------------*\
| Command
\*----------------------------------------------------------*/

void command(double velocity)
{
  double commandedVelocity = clamp(abs(velocity), minVelocity, maxVelocity);

  uint16_t dutyCycle = (uint16_t)map(
    commandedVelocity, minVelocity, maxVelocity, (double)minPwm, (double)maxPwm);

  lpwmCommand = (velocity >= 0 ? 0 : dutyCycle);
  rpwmCommand = (velocity >= 0 ? dutyCycle : 0);

  str1ker::Pwm msg;
  msg.channels.resize(2);

  // LPWM
  msg.channels[0].channel = lpwm;
  msg.channels[0].mode = str1ker::PwmChannel::MODE_ANALOG;
  msg.channels[0].value = lpwmCommand;
  msg.channels[0].duration = 0;

  // RPWM
  msg.channels[1].channel = rpwm;
  msg.channels[1].mode = str1ker::PwmChannel::MODE_ANALOG;
  msg.channels[1].value = rpwmCommand;
  msg.channels[1].duration = 0;

  pwmPub.publish(msg);
}

/*----------------------------------------------------------*\
| Listen to joint trajectory commands
\*----------------------------------------------------------*/

void control(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
  auto trajectory = msg->goal.trajectory;

  // Find the joint index we are listening for
  auto jointPos = std::find(trajectory.joint_names.cbegin(), trajectory.joint_names.cend(), joint);

  if (jointPos != trajectory.joint_names.cend())
  {
    // Build steps from the trajectory
    size_t jointIndex = jointPos - trajectory.joint_names.cbegin();
    std::vector<step> trajectorySteps(trajectory.points.size());
    double prevTime = 0.0;

    for (int n = 0; n < trajectorySteps.size(); n++)
    {
      trajectorySteps[n].velocity = trajectory.points[n].velocities[jointIndex];

      double pointTime = trajectory.points[n].time_from_start.toSec();
      trajectorySteps[n].duration = pointTime - prevTime;
      prevTime = pointTime;
    }

    steps = trajectorySteps;

    command(velocity);
  }
  else
  {
    ROS_WARN("joint %s not found in trajectory", joint.c_str());
  }
}

/*----------------------------------------------------------*\
| Feedback
\*----------------------------------------------------------*/

void feedback(const str1ker::Adc::ConstPtr& msg)
{
  // Read analog input
  reading = (*pFilter)(msg->adc[input]);

  // Re-map to position
  position = map((double)reading, (double)minReading, (double)maxReading, minPos, maxPos);
}

/*----------------------------------------------------------*\
| Recording
\*----------------------------------------------------------*/

void record(double elapsed)
{
  if (!outputLog.size()) return;

  logFile << elapsed << ", "
          << velocity << ", "
          << lpwmCommand << ", "
          << rpwmCommand << ", "
          << position << ", "
          << reading << std::endl;

  ROS_INFO(
    "[%d] time %#.4g\tvel %#+.4g\t\tLPWM %3d\tRPWM %3d\tpos %#.4g\treading %4d",
    int(currentStep + 1),
    elapsed,
    velocity,
    int(lpwmCommand),
    int(rpwmCommand),
    position,
    reading
  );
}

/*----------------------------------------------------------*\
| Utilities
\*----------------------------------------------------------*/

template<class T> T clamp(T value, T min, T max)
{
  if (value < min && abs(value) > std::numeric_limits<double>::epsilon()) return min;
  if (value > max) return max;

  return value;
}

template<class T> T map(T value, T min, T max, T targetMin, T targetMax)
{
  if (abs(value) > std::numeric_limits<double>::epsilon())
  {
    return (value - min) / (max - min) * (targetMax - targetMin) + targetMin;
  }
  else
  {
    return 0;
  }
}
