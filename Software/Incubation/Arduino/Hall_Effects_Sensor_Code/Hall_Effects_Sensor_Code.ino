int SensorInput = 7;//Pin used for sensor FR
//int SensorInput = 8;//Pin used for sensor FL
//int SensorInput = 9;//Pin used for sensor BR
//int SensorInput = 10;//Pin used for sensor BL

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>
ros::NodeHandle nh;

//ROS node and publisher setup
std_msgs::Int16MultiArray align_msg;
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
  //Serial.begin(9600);
  align_msg.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension) * 1);
  align_msg.layout.dim[0].label = dim0_label;
  align_msg.layout.dim[0].size = 4; 
  align_msg.layout.dim[0].stride = 1*4;
  align_msg.layout.data_offset = 0;
  align_msg.data_length = 4;
  align_msg.data = (int *)malloc(sizeof(int)*4);
  nh.advertise(pub_align);
  //SensorInit(SensorInputFR,SensorOutputFR);
  //SensorInit(SensorInputFL,SensorOutputFL);
  //SensorInit(SensorInputBR,SensorOutputBR);
  //SensorInit(SensorInputBL,SensorOutputBL);
  pinMode(SensorInput, INPUT);
  pinMode(13, OUTPUT);
}

void SensorInit(int sensor_input,int sensor_output){
  pinMode(sensor_input, INPUT);
  pinMode(sensor_output, OUTPUT);
  return;
}

void loop()
{
  if (to_calibrate)
  {
    digitalWrite(13,digitalRead(SensorInput));
    align_msg.data[0] = (digitalRead(SensorInput));//Sends 1 if aligned 0 if not.
    align_msg.data[1] = 1;
    align_msg.data[2] = 1;
    align_msg.data[3] = 1;
  }
  else
  {
    align_msg.data[0] = 1;
    align_msg.data[1] = 1;
    align_msg.data[2] = 1;
    align_msg.data[3] = 1;
  }
  pub_align.publish( &align_msg );
  nh.spinOnce();
  delay(80); 
}


