/*
                                   
  ██ ██  █████ █████ █████ █████         █  █████  █████   ████ █████ █████ ████    ████ █████
 █  █  █ █   █   █   █   █ █    █        █  █    █ █      █     █   █ █   █ █   █  █     █
 █  █  █ █   █   █   █   █ █████        █   █████  ████    ██   █████ █   █ █   █   ██   ████
 █  █  █ █   █   █   █   █ █    █   ████    █    █ █         █  █     █   █ █   █     █  █
 █  █  █ █████   █   █████ █    █  █        █    █ █████ ████   █     █████ █   █ ████   █████
                                  █
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
#include <motor_response/Adc.h>       // Analog read request
#include <motor_response/Pwm.h>       // Analog write request
#include <QuadratureEncoder.h>        // QuadratureEncoder library

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

// ROS topics and spin rate
const char ADC_TOPIC[] = "adc";
const char PWM_TOPIC[] = "pwm";
const double RATE_HZ = 50.0;
const int DELAY = 1000.0 / RATE_HZ;

// Motor PWM outputs
const int PWM_CHANNELS = 6;
const int PWM_PINS[] =
{
  3,
  11,
  5,
  6,
  9,
  10
};

// Absolute encoder inputs
const int ANALOG_CHANNELS = 7;
const int ANALOG_PINS[] =
{
  A0,
  A1,
  A2,
  A3,
  A4,
  A5,
  A6
};

// Relative encoder inputs
const int QUADRATURE_CHANNELS = 1;
const int QUADRATURE_A = 2;
const int QUADRATURE_B = 7;

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

void writePwm(const motor_response::Pwm& msg);

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

// Absolute and Relative encoder readings
int16_t adc[ANALOG_CHANNELS + QUADRATURE_CHANNELS] = {0};

// Relative encoder
Encoders encoder(QUADRATURE_A, QUADRATURE_B);

// ADC publisher
motor_response::Adc msg;
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
  for (int channel = 0; channel < PWM_CHANNELS; channel++)
  {
    pinMode(PWM_PINS[channel], OUTPUT);
  }

  node.subscribe(sub);
}

void setup()
{
  node.initNode();
  initAdc();
  initPwm();

  // Ensure the node is ready
  node.negotiateTopics();
  delay(3000);
}

/*----------------------------------------------------------*\
| Input
\*----------------------------------------------------------*/

void readAdc()
{
  for (int channel = 0; channel < ANALOG_CHANNELS; channel++)
  {
    adc[channel] = (int16_t)analogRead(ANALOG_PINS[channel]);
  }

  adc[ANALOG_CHANNELS] = encoder.getEncoderCount();

  msg.adc_length = ANALOG_CHANNELS + QUADRATURE_CHANNELS;
  msg.adc = adc;
    
  pub.publish(&msg);
}

/*----------------------------------------------------------*\
| Output
\*----------------------------------------------------------*/

void writePwm(const motor_response::Pwm& msg)
{
  for (int n = 0; n < msg.channels_length; n++)
  {
    motor_response::PwmChannel& request = msg.channels[n];

    if (request.channel >= 0 && request.channel < PWM_CHANNELS)
    {
      analogWrite(PWM_PINS[request.channel], request.value);
    }
  }
}

/*----------------------------------------------------------*\
| Processing
\*----------------------------------------------------------*/

void loop()
{
  readAdc();
  node.spinOnce();
  delay(DELAY);
}
