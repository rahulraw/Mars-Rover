int SensorInput = 7;//Pin used for sensor FR
//int SensorInput = 8;//Pin used for sensor FL
//int SensorInput = 9;//Pin used for sensor BR
//int SensorInput = 10;//Pin used for sensor BL

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
ros::NodeHandle nh;

//ROS node and publisher setup
std_msgs::Int16 align_msg;
ros::Publisher pub_align("Aligned", &align_msg);
char dim0_label[] = "Aligned";

boolean to_calibrate;
String calibrate;

void messageCb( const std_msgs::Bool& calibrate_call){
  to_calibrate = calibrate_call.data;
}

ros::Subscriber<std_msgs::Bool> sub("calibration", messageCb);

void setup()
{
  nh.initNode();
  Serial.begin(9600);
  //SensorInit(SensorInputFR,SensorOutputFR);
  //SensorInit(SensorInputFL,SensorOutputFL);
  //SensorInit(SensorInputBR,SensorOutputBR);
  //SensorInit(SensorInputBL,SensorOutputBL);
  pinMode(SensorInput, INPUT);
  pinMode(13, OUTPUT);
  nh.advertise(pub_align);
}

void SensorInit(int sensor_input,int sensor_output) {
  pinMode(sensor_input, INPUT);
  pinMode(sensor_output, OUTPUT);
  return;
}

void loop()
{
  if (to_calibrate)
  {
    digitalWrite(13,digitalRead(SensorInput));
    align_msg.data = (digitalRead(SensorInput));//Sends 1 if aligned 0 if not.
  }
  else
  {
    align_msg.data = 1;
  }
  pub_align.publish( &align_msg );
  nh.spinOnce();
  delay(80); 
}


