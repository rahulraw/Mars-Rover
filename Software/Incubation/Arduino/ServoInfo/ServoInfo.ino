//#include <ArduinoHardware.h>
#include <Servo.h>

//do a half half voltage divider w/ green=GND, red=+5 yellow=FB
#define PIN_CAM_ROTATE  10//backward/forward controlled PWM
#define CAM_ROTATE_STOP 1354
#define CAM_ROTATE_CW   CAM_ROTATE_STOP + 50
#define CAM_ROTATE_CCW  CAM_ROTATE_STOP - 50

#define PIN_CAM_PITCH   3//position controlled PWM

Servo srvCamRot;

int readValue = 0;

void setup(){
    Serial.begin(9600);
    srvCamRot.attach(PIN_CAM_ROTATE, 771, 1798);
}

void loop(){
    while(Serial.available()) {
        readValue = Serial.parseInt();
    }

    Serial.println(readValue, DEC);
    srvCamRot.writeMicroseconds(readValue);
    delay(1000);
}


