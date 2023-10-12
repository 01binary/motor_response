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
#include <std_msgs/Float64.h>
#include <control_toolbox/pid.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <angles/angles.h>
#include <urdf/model.h>

//
// Helpers
//

#include "motor.h"
#include "encoder.h"
#include "utilities.h"

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

enum supportedJointTypes
{
  REVOLUTE = 1,
  PRISMATIC = 3
};

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
std::string trajectoryTopic;

// Input topic for listening to position commands
std::string commandTopic;

// The joint to watch for in trajectory commands
std::string joint;

// The joint type extracted from robot_description
supportedJointTypes jointType = REVOLUTE;

// Upper joint limit extracted from robot_description
double upperLimit;

// Lower joint limit extracted form robot_description
double lowerLimit;

// The default goal position tolerance
double defaultTolerance;

//
// State
//

// Current trajectory
std::vector<trajectoryPoint> trajectory;

// Current trajectory point
int point = -1;

// Current trajectory tolerance
double goalTolerance;

// Current trajectory point start time
ros::Time startTime;

// Last loop time
ros::Time lastTime;

// Last command
double lastCommand = 0.0;

// Last position
double lastPos = 0.0;

// Done executing trajectory
bool done = true;

// Actuator for sending velocity commands
motor actuator;

// Sensor for reading position
encoder sensor;

// Subscriber for receiving joint trajectories
ros::Subscriber trajSub;

// Subscriber for receiving joint command
ros::Subscriber cmdSub;

// Controller for converting trajectories to commands
control_toolbox::Pid pid;

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

void configure();
void initialize(ros::NodeHandle node);
void commandControl(const std_msgs::Float64::ConstPtr& msg);
void trajectoryControl(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg);
void beginTrajectory(ros::Time time, std::vector<trajectoryPoint> points, double tolerance);
void endTrajectory();
void runTrajectory(ros::Time time);

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
      runTrajectory(ros::Time::now());
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
  ros::param::get("trajectoryTopic", trajectoryTopic);
  ros::param::get("commandTopic", commandTopic);
  ros::param::get("joint", joint);
  ros::param::get("tolerance", defaultTolerance);

  // Read actuator settings
  actuator.configure();

  // Read sensor settings
  sensor.configure();

  // Read joint settings
  urdf::Model urdf;

  if (urdf.initParam("robot_description"))
  {
    auto jointDescription = urdf.getJoint(joint);

    if (jointDescription)
    {
      if (jointDescription->type == urdf::Joint::REVOLUTE)
      {
        ROS_INFO("joint %s is revolute", joint.c_str());
        jointType = REVOLUTE;
      }
      else if (jointDescription->type == urdf::Joint::PRISMATIC)
      {
        ROS_INFO("joint %s is prismatic", joint.c_str());
        jointType = PRISMATIC;
      }
      else
      {
        ROS_WARN("joint %s is not revolute or prismatic", joint.c_str());
      }

      upperLimit = jointDescription->limits->upper;
      lowerLimit = jointDescription->limits->lower;
    }
    else
    {
      ROS_WARN("joint %s not found in robot description", joint.c_str());
    }
  }
  else
  {
    jointType = REVOLUTE;
    lowerLimit = sensor.getMin();
    upperLimit = sensor.getMax();
  }
}

/*----------------------------------------------------------*\
| Initialization
\*----------------------------------------------------------*/

void initialize(ros::NodeHandle node)
{
  // Initialize joint trajectory subscriber
  trajSub = node.subscribe<control_msgs::FollowJointTrajectoryActionGoal>(
    trajectoryTopic, 1, &trajectoryControl);

  // Initialize joint command subscriber
  trajSub = node.subscribe<std_msgs::Float64>(
    commandTopic, 1, &commandControl);

  // Initialize sensor
  sensor.initialize(node);

  // Initialize actuator
  actuator.initialize(node);

  // Initialize PID controller
  if (!pid.init(ros::NodeHandle(node, "pid")))
    exit(1);

  pid.printValues();
}

/*----------------------------------------------------------*\
| Control
\*----------------------------------------------------------*/

void commandControl(const std_msgs::Float64::ConstPtr& msg)
{
  beginTrajectory(
    ros::Time::now(),
    {
      { msg->data, 1.0, 1.0 }
    },
    defaultTolerance
  );
}

void trajectoryControl(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
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

    double tolerance;

    if (msg->goal.goal_tolerance.size() > jointIndex)
    {
      tolerance = msg->goal.goal_tolerance[jointIndex].position;
    }
    else
    {
      tolerance = defaultTolerance;
    }

    beginTrajectory(ros::Time::now(), trajectoryPoints, tolerance);
  }
  else
  {
    ROS_WARN("joint %s not found in trajectory", joint.c_str());
  }
}

/*----------------------------------------------------------*\
| Trajectory playback
\*----------------------------------------------------------*/

void beginTrajectory(ros::Time time, std::vector<trajectoryPoint> points, double tolerance)
{
  ROS_INFO(
    "starting trajectory with %d points %g tolerance",
    (int)points.size(),
    tolerance
  );

  startTime = time;
  point = 0;
  trajectory = points;
  goalTolerance = tolerance;
  done = false;
}

void endTrajectory()
{
  ROS_INFO("trajectory completed");

  actuator.command(0.0);
  done = true;
}

void runTrajectory(ros::Time time)
{
  if (!trajectory.size() || done)
  {
    return;
  }

  ros::Duration elapsed = time - startTime;
  bool isLast = point == int(trajectory.size()) - 1;

  if (elapsed.toSec() >= trajectory[point].duration && !isLast)
  {
    point++;
  }

  // Mimic the behavior of joint_position_controller from ros_controllers
  ros::Duration period = time - lastTime;
  double velocity = trajectory[point].velocity;
  double position = trajectory[point].position;
  double velocityError = velocity - actuator.getVelocity();
  double positionError;
  
  if (jointType == REVOLUTE)
  {
    angles::shortest_angular_distance_with_large_limits(
      sensor.getPosition(),
      position,
      lowerLimit,
      upperLimit,
      positionError);
  }
  else if (jointType == PRISMATIC)
  {
    positionError = position - sensor.getPosition();
  }

  if (abs(positionError) <= goalTolerance && isLast)
  {
    endTrajectory();
    return;
  }

  double command = pid.computeCommand(positionError, velocityError, period);

  ROS_INFO(
    "[%d] time %#.4g\tper %#.4g\ttarget pos %#+.4g\tpos %#+.4g\ttarget vel %#+.4g\tvel %#+.4g\tperr %#+.4g\tverr %#+.4g",
    int(point),
    elapsed.toSec(),
    period.toSec(),
    position,
    sensor.getPosition(),
    velocity,
    actuator.getVelocity(),
    positionError,
    velocityError
  );

  if (point > 0 &&
      !isSameSign(command, lastCommand) &&
      isSameSign(lastCommand, trajectory.back().position - sensor.getPosition()))
  {
    // Do not reverse direction if we missed a waypoint as long as we're moving toward goal
    ROS_WARN("prevented reversing");
    return;
  }

  actuator.command(command);

  lastCommand = command;
  lastTime = time;
}
