#include "CameraNode.h"

CameraNode::CameraNode(int pulse_width_min1, int pulse_width_max1, int pulse_width_min2, int pulse_width_max2)
{
    this->servoYaw(pulse_width_min1, pulse_width_max1);
    this->servoPitch(pulse_width_min2, pulse_width_max2);
}

void CameraNode::run(char * data)
{
    this->servoYaw.run(data[0]);
    this->servoPitch.run(data[1]);
}
