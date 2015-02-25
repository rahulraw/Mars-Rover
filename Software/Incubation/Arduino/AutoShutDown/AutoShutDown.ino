int RelayPin = 4;

#include <ros.h>
#include <std_msgs/Bool.h>
ros::NodeHandle nh;

boolean turn_off;

void call_back( const std_msgs::Bool& off){
  turn_off = off.data;
}

ros::Subscriber<std_msgs::Bool> sub("shutoff", &call_back);

void setup()
{
  nh.initNode();
  Serial.begin(9600);
  pinMode(RelayPin, OUTPUT);
  turn_off = false;
}

void loop()
{
  if (turn_off) 
  {
    digitalWrite(RelayPin, LOW);
  }
  else 
  {
    digitalWrite(RelayPin, HIGH);
  }
  nh.spinOnce();
  delay(80); 
}


