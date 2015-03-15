#include "CameraNode.h"
#include "Arduino.h"

#define CAM_ROTATE_STOP 1354
#define CAM_ROTATE_CW   CAM_ROTATE_STOP + 30
#define CAM_ROTATE_CCW  CAM_ROTATE_STOP - 30

#define COMMAND_STOP    1
#define COMMAND_LOW     0
#define COMMAND_HIGH    2

#define PIN_CAM_ZOOM    8

CameraNode::CameraNode(int pulse_width_min1, int pulse_width_max1, int pulse_width_min2, int pulse_width_max2)
{
    this->servoRotate = new ServoNode(pulse_width_min2, pulse_width_max1);
    this->servoPitch = new ServoNode(pulse_width_min2, pulse_width_max2);

    this->bytes = 3;
}

void CameraNode::run(char * data)
{
    char rotate = data[0];
    char pitch = data[1];
    char zoom = data[2];

    servoPitch->run(&pitch);

    if (rotate == COMMAND_STOP){
        servoPitch->writeMicroseconds(CAM_ROTATE_STOP);
    }
    else if (rotate == COMMAND_LOW){
        servoPitch->writeMicroseconds(CAM_ROTATE_CCW);
    }
    else if (rotate == COMMAND_HIGH){
        servoPitch->writeMicroseconds(CAM_ROTATE_CW);
    }

    if (zoom == COMMAND_STOP){
        pinMode(PIN_CAM_ZOOM, INPUT);
    }
    else if (zoom == COMMAND_LOW){
        pinMode(PIN_CAM_ZOOM, OUTPUT);
        digitalWrite(PIN_CAM_ZOOM, LOW);
    }
    else if (zoom == COMMAND_HIGH){
        pinMode(PIN_CAM_ZOOM, OUTPUT);
        analogWrite(PIN_CAM_ZOOM, 255);
    }

    Serial.write(data[0]);
    Serial.write(data[1]);
    Serial.write(data[2]);
}

CameraNode::~CameraNode() {
    delete this->servoRotate;
    delete this->servoPitch;
}
