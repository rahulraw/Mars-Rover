#include <ros.h>
#include <std_msgs/Bool.h>
#include "AutoShutDown.h"

ros::NodeHandle nh;

void setup()
{
  nh.initNode();
  Serial.begin(9600);
}

void loop()
{
  nh.spinOnce();
  delay(80); 
}


