//#include <ArduinoHardware.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Empty.h>

//do a half half voltage divider w/ green=GND, red=+5 yellow=FB
#define PIN_CAM_ROTATE  2//backward/forward controlled PWM
#define CAM_ROTATE_STOP 1354
#define CAM_ROTATE_CW   CAM_ROTATE_STOP + 50
#define CAM_ROTATE_CCW  CAM_ROTATE_STOP - 50

#define PIN_CAM_PITCH   3//position controlled PWM

Servo srvCamRot, srvCamPitch;

ros::NodeHandle nh;

void rosCallback(const std_msgs::Empty& msg){
    
}

ros::Subscriber<std_msgs::Empty> rosSub("toggle_led", &rosCallback );

void setup(){
    srvCamRot.attach(PIN_CAM_ROTATE);
    srvCamRot.writeMicroseconds(CAM_ROTATE_STOP);

    camRotateCCW();

    srvCamPitch.attach(PIN_CAM_PITCH);
    camPitch(125);


    nh.initNode();
    nh.subscribe(rosSub);
}

void loop(){
    // camRotateCCW();
    camPitch(110);
    delay(1000);
    // camRotateCW();
    camPitch(140);
    delay(1000);
}

void camRotateCW(){
    srvCamRot.writeMicroseconds(CAM_ROTATE_CW);
}

void camRotateCCW(){
    srvCamRot.writeMicroseconds(CAM_ROTATE_CCW);
}

void camPitch(int angle){
    srvCamPitch.write(angle);
}
