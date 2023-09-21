/*
  motor_response.cpp

  DC motor characterizatoin tool
  Created 09/20/2023

  Copyright (C) 2023 Valeriy Novytskyy
  This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include <str1ker/Pwm.h>

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

/*----------------------------------------------------------*\
| Package entry point
\*----------------------------------------------------------*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot");

    ros::NodeHandle node;
    
    // initialize here

    return 0;
}
