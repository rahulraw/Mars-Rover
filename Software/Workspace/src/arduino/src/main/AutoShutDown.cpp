
#include "Arduino.h"
#include "AutoShutDown.h"

void AutoShutDown::messageCb(const std_msgs::Bool& off)
{
  this->turn_off = off.data;
}

void AutoShutDown::AutoShutOff(ros::NodeHandle nh)
{
  ros::Subscriber<std_msgs::Bool> sub("shutoff", AutoShutDown::messageCb);
  //this->sub = ros::Subscriber<std_msgs::Bool> ("shutoff", AutoShutDown::callback);
  nh.subscribe(sub);
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


