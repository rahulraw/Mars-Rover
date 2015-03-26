#include <ArduinoHardware.h>
#include <Servo.h>

//do a half half voltage divider w/ green=GND, red=+5 yellow=FB
#define PIN_CAM_ROTATE  2//backward/forward controlled PWM
#define CAM_ROTATE_STOP 1360
#define CAM_ROTATE_CW   CAM_ROTATE_STOP + 50
#define CAM_ROTATE_CCW  CAM_ROTATE_STOP - 50

#define PIN_CAM_PITCH   3//position controlled PWM
#define PIN_CAM_ZOOM    8

Servo srvCamRot, srvCamPitch;

void setup(){
    srvCamRot.attach(PIN_CAM_ROTATE);
    srvCamRot.writeMicroseconds(CAM_ROTATE_STOP);

    // camRotateCCW();
    Serial.begin(9600);

    srvCamPitch.attach(PIN_CAM_PITCH);

    char v = 140;

    Serial.println((uint8_t) v);
    //srvCamPitch.write((int) v);
    //camPitch(160);

    // pinMode(PIN_CAM_ZOOM, OUTPUT);
    // digitalWrite(PIN_CAM_ZOOM, LOW);
    // delay(1000);
    // pinMode(PIN_CAM_ZOOM, INPUT);
    // delay(1000);

    // pinMode(PIN_CAM_ZOOM, OUTPUT);
    // analogWrite(PIN_CAM_ZOOM, 255);
    // delay(1000);
}

void loop(){
    // // camRotateCCW();
    // camPitch(120);
    // delay(1000);
    // // camRotateCW();
    // camPitch(130);
    // delay(1000);

    // pinMode(PIN_CAM_ZOOM, OUTPUT);
    // digitalWrite(PIN_CAM_ZOOM, LOW);
    // delay(2000);
    // pinMode(PIN_CAM_ZOOM, INPUT);
    // delay(2000);

    // pinMode(PIN_CAM_ZOOM, OUTPUT);
    // analogWrite(PIN_CAM_ZOOM, 255);
    // delay(2000);
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

void camZoomIn(){

}

void camZoomOut(){

}