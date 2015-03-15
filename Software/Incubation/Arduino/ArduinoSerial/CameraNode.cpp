#include "CameraNode.h"
#include "Arduino.h"

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

    if (pitch < CAM_PITCH_LOW_LIMIT){
        pitch = CAM_PITCH_LOW_LIMIT;
    }
    else if (pitch > CAM_PITCH_HIGH_LIMIT){
        pitch = CAM_PITCH_HIGH_LIMIT;
    }

    servoPitch->servo.write(pitch);

    if (rotate == COMMAND_STOP){
        servoRotate->servo.writeMicroseconds(CAM_ROTATE_STOP);
    }
    else if (rotate == COMMAND_LOW){
        servoRotate->servo.writeMicroseconds(CAM_ROTATE_CCW);
    }
    else if (rotate == COMMAND_HIGH){
        servoRotate->servo.writeMicroseconds(CAM_ROTATE_CW);
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
