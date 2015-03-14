#include "CameraNode.h"

CameraNode::CameraNode(int pulse_width_min1, int pulse_width_max1, int pulse_width_min2, int pulse_width_max2)
{
    this->servoYaw = new ServoNode(pulse_width_min2, pulse_width_max1);
    this->servoPitch = new ServoNode(pulse_width_min2, pulse_width_max2);

    this->bytes = 3;
}

void CameraNode::run(char * data)
{
    this->servoYaw->run(&data[0]);
    this->servoPitch->run(&data[1]);
    this->zoom = data[0];
}

CameraNode::~CameraNode() {
    delete this->servoYaw;
    delete this->servoPitch;
}
