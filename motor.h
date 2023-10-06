/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include <ros/ros.h>
#include <str1ker/Pwm.h>
#include "utilities.h"

/*----------------------------------------------------------*\
| motor class
\*----------------------------------------------------------*/

class motor
{
private:
  // Default min PWM pulse width
  const int PWM_MIN = 0;

  // Default max PWM pulse width
  const int PWM_MAX = 4096;

  // Publish queue size
  const int QUEUE_SIZE = 8;

private:
  //
  // Configuration
  //

  // PWM output topic
  std::string m_topic = "robot/pwm";

  // Left PWM channel
  int m_lpwm = 0;

  // Right PWM channel
  int m_rpwm = 1;

  // Min PWM pulse width
  int m_minPwm = PWM_MIN;

  // Max PWM pulse width
  int m_maxPwm = PWM_MAX;

  // Min velocity
  double m_minVelocity = 0.0;

  // Max velocity
  double m_maxVelocity = 1.0;

  //
  // Interface
  //

  // PWM publisher to motor driver
  ros::Publisher m_pwmPub;

  //
  // State
  //

  // Last LPWM pulse width
  uint16_t m_lpwmCommand = 0;

  // Last RPWM pulse width
  uint16_t m_rpwmCommand = 0;

  // Last velocity command
  double m_velocity = 0.0;

public:
  //
  // Constructor
  //

  motor()
  {
  }

  motor(
    std::string topic, int lpwm, int rpwm, int minPwm, int maxPwm, double minVelocity, double maxVelocity)
    : m_topic(topic)
    , m_lpwm(lpwm)
    , m_rpwm(rpwm)
    , m_minPwm(minPwm)
    , m_maxPwm(maxPwm)
    , m_minVelocity(minVelocity)
    , m_maxVelocity(maxVelocity)
  {
  }

public:
  inline double velocity() {
    // Get current velocity
    return m_velocity;
  }

  inline int lpwm() {
    // Get current LPWM pulse width
    return m_lpwmCommand;
  }

  inline int rpwm() {
    // Get current RPWM pulse width
    return m_rpwmCommand;
  }

  void configure()
  {
    // Read configuration settings
    ros::param::get("outputTopic", m_topic);
    ros::param::get("lpwm", m_lpwm);
    ros::param::get("rpwm", m_rpwm);
    ros::param::get("minPwm", m_minPwm);
    ros::param::get("maxPwm", m_maxPwm);
    ros::param::get("minVelocity", m_minVelocity);
    ros::param::get("maxVelocity", m_maxVelocity);
  }

  void initialize(ros::NodeHandle node)
  {
    // Initialize PWM output publisher
    m_pwmPub = node.advertise<str1ker::Pwm>(
      m_topic,
      QUEUE_SIZE
    );
  }
  
  void command(double velocity)
  {
    m_velocity = clampZero(abs(velocity), m_minVelocity, m_maxVelocity);

    uint16_t dutyCycle = (uint16_t)mapZero(
      m_velocity, m_minVelocity, m_maxVelocity, (double)m_minPwm, (double)m_maxPwm);

    m_lpwmCommand = (velocity >= 0 ? 0 : dutyCycle);
    m_rpwmCommand = (velocity >= 0 ? dutyCycle : 0);

    str1ker::Pwm msg;
    msg.channels.resize(2);

    // LPWM
    msg.channels[0].channel = m_lpwm;
    msg.channels[0].mode = str1ker::PwmChannel::MODE_ANALOG;
    msg.channels[0].value = m_lpwmCommand;
    msg.channels[0].duration = 0;

    // RPWM
    msg.channels[1].channel = m_rpwm;
    msg.channels[1].mode = str1ker::PwmChannel::MODE_ANALOG;
    msg.channels[1].value = m_rpwmCommand;
    msg.channels[1].duration = 0;

    m_pwmPub.publish(msg);
  }
};
