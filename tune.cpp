/*
  tune.cpp

  Tune DC motor PID controller
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

#include <string>
#include <vector>

//
// ROS
//

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

//
// Helpers
//

#include "motor.h"
#include "encoder.h"

/*----------------------------------------------------------*\
| Types
\*----------------------------------------------------------*/

struct trajectoryPoint
{
  double position;
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

//
// State
//

// Current trajectory
std::vector<trajectoryPoint> trajectory;

// Current trajectory point
size_t point = 0;

// Current trajectory point start time
ros::Time start;

// Actuator for sending velocity commands
motor actuator;

// Sensor for reading position
encoder sensor;

// Subscriber for receiving joint trajectories
ros::Subscriber sub;

// Controller for converting trajectories to commands
control_toolbox::Pid pid;

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

void configure();
void initialize(ros::NodeHandle node);
void control(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg);
void playback(ros::Time time);

/*----------------------------------------------------------*\
| Package entry point
\*----------------------------------------------------------*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tune");
  ros::NodeHandle node;

  // Configure node
  configure();

  // Initialize node
  initialize(node);

  // Run node
  ros::Rate rate(spinRate);

  while(node.ok())
  {
    if (!sensor.isReady())
    {
      // Wait until we have readings from the encoder
      ROS_INFO("waiting for feedback...");
    }
    else
    {
      // Execute trajectory
      playback(ros::Time::now());
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
  // Read control settings
  ros::param::get("rate", spinRate);
  ros::param::get("controlTopic", controlTopic);
  ros::param::get("joint", joint);

  // Read actuator settings
  actuator.configure();

  // Read sensor settings
  sensor.configure();
}

/*----------------------------------------------------------*\
| Initialization
\*----------------------------------------------------------*/

void initialize(ros::NodeHandle node)
{
  // Initialize joint trajectory subscriber
  sub = node.subscribe<control_msgs::FollowJointTrajectoryActionGoal>(
    controlTopic, 1, &control);

  // Initialize sensor
  sensor.initialize(node);

  // Initialize actuator
  actuator.initialize(node);

  // Initialize PID controller
  if (!pid.init(ros::NodeHandle(node, "pid")))
    exit(1);
}

/*----------------------------------------------------------*\
| Playback
\*----------------------------------------------------------*/

void playback(ros::Time time)
{
  if (!trajectory.size()) return;

  ros::Duration period = time - start;

  if (period.toSec() >= trajectory[point].duration)
  {
    // Go to the next step
    point = (point + 1) % trajectory.size();
    start = time;

    // Calculate PID
    double velocity = trajectory[point].velocity;
    double position = trajectory[point].position;
    double positionError = position - sensor.getPosition();
    double velocityError = velocity - actuator.getVelocity();

    double command = pid.computeCommand(
      positionError, velocityError, period);

    actuator.command(command);
  }
}

/*----------------------------------------------------------*\
| Trajectory control
\*----------------------------------------------------------*/

void control(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
  auto goalTrajectory = msg->goal.trajectory;

  // Find the joint index we are listening for
  auto jointPos = std::find(
    goalTrajectory.joint_names.cbegin(),
    goalTrajectory.joint_names.cend(),
    joint
  );

  if (jointPos != goalTrajectory.joint_names.cend())
  {
    // Parse trajectory
    size_t jointIndex = jointPos - goalTrajectory.joint_names.cbegin();
    std::vector<trajectoryPoint> trajectoryPoints(goalTrajectory.points.size());
    double prevTime = 0.0;

    for (int n = 0; n < trajectoryPoints.size(); n++)
    {
      trajectoryPoints[n].position = goalTrajectory.points[n].positions[jointIndex];
      trajectoryPoints[n].velocity = goalTrajectory.points[n].velocities[jointIndex];

      double timeFromStart = goalTrajectory.points[n].time_from_start.toSec();
      trajectoryPoints[n].duration = timeFromStart - prevTime;
      prevTime = timeFromStart;
    }

    trajectory = trajectoryPoints;
  }
  else
  {
    ROS_WARN("joint %s not found in trajectory", joint.c_str());
  }
}
