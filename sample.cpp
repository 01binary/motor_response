/*
  sample.cpp

  Record DC motor response to step input(s)
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
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

//
// ROS
//

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

//
// Helpers
//

#include "motor.h"
#include "encoder.h"

/*----------------------------------------------------------*\
| Types
\*----------------------------------------------------------*/

struct step
{
  double velocity;
  double duration;
};

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

//
// Configuration
//

// Update rate
int spinRate;

// Input topic for listening to trajectory commands
std::string controlTopic;

// The joint to watch for in trajectory commands
std::string joint;

// Output log CSV file for recording commands and readings
std::string outputLog;

// Steps to play back (velocity, duration)
std::vector<step> steps;

//
// State
//

// Opened log file for recording commands and readings
std::ofstream logFile;

// Current step being played back and when it started
size_t currentStep = 0;
ros::Time stepTime;

// Actuator for sending velocity commands
motor actuator;

// Sensor for reading position
encoder sensor;

// Subscriber for receiving follow joint trajectory commands
ros::Subscriber controlSub;

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

void configure();
void initialize(ros::NodeHandle node);
void control(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg);
void playback(ros::Time time);
void record(double elapsed);

/*----------------------------------------------------------*\
| Package entry point
\*----------------------------------------------------------*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample");
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
    if (!sensor.ready())
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
  // Read playback and control settings
  ros::param::get("rate", spinRate);
  ros::param::get("controlTopic", controlTopic);
  ros::param::get("joint", joint);
  ros::param::get("log", outputLog);

  // Read actuator settings
  actuator.configure();

  // Read sensor settings
  sensor.configure();

  // Read steps
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
  // Initialize joint trajectory subscriber if specified
  if (controlTopic.size())
  {
    controlSub = node.subscribe<control_msgs::FollowJointTrajectoryActionGoal>(
      controlTopic, 1, &control);
  }

  // Open the log file if specified
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
    actuator.command(steps[currentStep].velocity);
    if (outputLog.size()) logFile.flush();
  }
}

/*----------------------------------------------------------*\
| Trajectory control
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
  }
  else
  {
    ROS_WARN("joint %s not found in trajectory", joint.c_str());
  }
}

/*----------------------------------------------------------*\
| Recording
\*----------------------------------------------------------*/

void record(double elapsed)
{
  if (!outputLog.size()) return;

  logFile << elapsed << ", "
          << actuator.velocity() << ", "
          << actuator.lpwm() << ", "
          << actuator.rpwm() << ", "
          << sensor.position() << ", "
          << sensor.reading()
          << std::endl;

  ROS_INFO(
    "[%d] time %#.4g\tvel %#+.4g\t\tLPWM %3d\tRPWM %3d\tpos %#.4g\treading %4d",
    int(currentStep + 1),
    elapsed,
    actuator.velocity(),
    actuator.lpwm(),
    actuator.rpwm(),
    sensor.position(),
    sensor.reading()
  );
}
