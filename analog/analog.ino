/*
 analog.ino

 Arduino Analog Controller
 Created 3/22/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#define USE_USBCON

#include <ros.h>                      // ROS communication
#include <Adafruit_PWMServoDriver.h>  // Analog write library
#include <motor_response/Adc.h>       // Analog read request
#include <motor_response/Pwm.h>       // Analog write request

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

// ROS topics and spin rate
const char ADC_TOPIC[] = "adc";
const char PWM_TOPIC[] = "pwm";
const double RATE_HZ = 50.0;
const int DELAY = 1000.0 / RATE_HZ;

// Analog output
const int PWM_CHANNELS = 16;
const int PWM_FREQ_HZ = 5000;
const int PWM_MAX = 4096;
const int PWM_PINS[] =
{
  // ?
}

// Analog input
const int ANALOG_CHANNELS = 12;
const int ANALOG_PINS[] =
{
  A0,
  A1,
  A2,
  A3,
  A4,
  A5,
  A6,
  A7,
  A8,
  A9,
  A10,
  A11
};

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

void writePwm(const motor_response::Pwm& msg);

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

// ADC publisher
motor_response::Adc msg;
uint16_t adc[ANALOG_CHANNELS] = {0};
ros::Publisher pub(ADC_TOPIC, &msg);

// PWM subscriber
ros::Subscriber<motor_response::Pwm> sub(PWM_TOPIC, writePwm);

// ROS node
ros::NodeHandle node;

/*----------------------------------------------------------*\
| Initialization
\*----------------------------------------------------------*/

void initAdc()
{
  for (int channel = 0; channel < ANALOG_CHANNELS; channel++)
  {
    pinMode(ANALOG_PINS[channel], INPUT_PULLUP);
  }

  node.advertise(pub);
}

void initPwm()
{
  // TODO: init Arduino PWM pins

  node.subscribe(sub);
}

void setup()
{
  node.initNode();
  initAdc();
  initPwm();
}

/*----------------------------------------------------------*\
| Analog input
\*----------------------------------------------------------*/

void readAdc()
{
  for (int channel = 0; channel < ANALOG_CHANNELS; channel++)
  {
    adc[channel] = (uint16_t)analogRead(ANALOG_PINS[channel]);
  }

  msg.adc_length = ANALOG_CHANNELS;
  msg.adc = adc;
    
  pub.publish(&msg);
}

/*----------------------------------------------------------*\
| Analog output
\*----------------------------------------------------------*/

void analog(int channel, int value)
{
  // channel, when on (0-4096), when off (0-4096)
  // TODO: set using Arduino
  if (value == 0)
    pwm.setPWM(channel, 0, PWM_MAX);
  else if (value == PWM_MAX)
    pwm.setPWM(channel, PWM_MAX, 0);
  else
    pwm.setPWM(channel, PWM_MAX - value, 0);
}

void writePwm(const motor_response::Pwm& msg)
{
  for (uint32_t n = 0; n < msg.channels_length; n++)
  {
    motor_response::PwmChannel& request = msg.channels[n];
    if (request.channel >= PWM_CHANNELS) continue;

    analog(request.channel, request.value);
  }
}

/*----------------------------------------------------------*\
| Message handling
\*----------------------------------------------------------*/

void loop()
{
  readAdc();
  node.spinOnce();
  delay(DELAY);
}
