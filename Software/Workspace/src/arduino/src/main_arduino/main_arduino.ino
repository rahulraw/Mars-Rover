#include <ros.h>
#include <std_msgs/Bool.h>
#include "AutoShutDown.h"

ros::NodeHandle nh;
void setup()
{
  nh.initNode();
  AutoShutDown autoshutdown(7);
  ros::Subscriber<const std_msgs::Bool> sub("shutoff", autoshutdown.messageCb));
  nh.subscribe(sub);
  Serial.begin(9600);
}

void loop()
{
  nh.spinOnce();
  delay(80); 
}


