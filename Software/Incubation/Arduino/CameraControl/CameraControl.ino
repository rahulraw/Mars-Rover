#include <Servo.h> 

//do a half half voltage divider w/ green=GND, red=+5 yellow=FB
#define PIN_CAM_ROTATE  2//backward/forward controlled PWM
#define CAM_ROTATE_STOP 1354
#define CAM_ROTATE_CW   CAM_ROTATE_STOP + 50
#define CAM_ROTATE_CCW  CAM_ROTATE_STOP - 50

#define PIN_CAM_PITCH   3//position controlled PWM

Servo srvCamRot, srvCamPitch;

void setup(){
    srvCamRot.attach(PIN_CAM_ROTATE);
    srvCamRot.writeMicroseconds(CAM_ROTATE_STOP);

    camRotateCCW();

    srvCamPitch.attach(PIN_CAM_PITCH);
    
}

void loop(){
    camRotateCCW();
    camPitch(60);
    delay(1000);
    camRotateCW();
    camPitch(120);
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
