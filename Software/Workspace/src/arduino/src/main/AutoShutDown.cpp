
#include "Arduino.h"
#include "AutoShutDown.h"

void AutoShutDown::callback(const std_msgs::Bool& off)
{
  this->turn_off = off.data;
}

void AutoShutDown::AutoShutOff(ros::NodeHandle nh)
{
  this->sub = ("shutoff", AutoShutDown::callback);

  nh.subscribe(this->sub);
}

void AutoShutDown::run()
{
  if (this->turn_off) 
  {
    digitalWrite(this->_pin, LOW);
  }
  else 
  {
    digitalWrite(this->_pin, HIGH);
  }
}


