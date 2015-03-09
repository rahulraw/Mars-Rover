//#include <ArduinoHardware.h>
#include <Servo.h>

//do a half half voltage divider w/ green=GND, red=+5 yellow=FB
#define PIN_CAM_ROTATE  10//backward/forward controlled PWM
#define CAM_ROTATE_STOP 1354
#define CAM_ROTATE_CW   CAM_ROTATE_STOP + DIFF
#define CAM_ROTATE_CCW  CAM_ROTATE_STOP - DIFF

#define PIN_CAM_PITCH   3//position controlled PWM
#define DIFF 50

Servo srvCamRot;

int readValue = 1000;
int currentValue = readValue;
int diff = 0;

void setup(){
    Serial.begin(9600);
    srvCamRot.attach(PIN_CAM_ROTATE);
}

void loop(){
    if (Serial.available()) {
        readValue = Serial.parseInt();
    }

    Serial.print(currentValue, DEC);
    Serial.print(" ");
    Serial.println(readValue, DEC);
    
    if ((readValue - currentValue) > -DIFF && (readValue - currentValue) < DIFF || readValue == 0)
    {
        diff = 0;
    }
    else if ((readValue - currentValue) > DIFF)
    {
        diff = DIFF;
    }
    else if ((readValue - currentValue) < DIFF)
    {
        diff = -DIFF;
    }
    currentValue = currentValue + diff;
    srvCamRot.writeMicroseconds(currentValue);
    delay(100);
}


