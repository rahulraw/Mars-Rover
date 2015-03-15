#include <math.h>

#include "Arduino.h"
#include "ServoNode.h"

ServoNode::ServoNode()
{
    this->init(1000, 2000);
}

ServoNode::ServoNode(int pulse_width_min, int pulse_width_max)
{
    this->init(pulse_width_min, pulse_width_max);
}

void ServoNode::run(char * data)
{
    int pulse = this->getPulse((int) data[0]);
    this->servo.writeMicroseconds(pulse);
}

int ServoNode::getPulse(int angle)
{
   return (int) floor(((this->pulse_width_max - this->pulse_width_min) / 90.0) * angle) + this->pulse_width_min;
}

void ServoNode::init(int pulse_width_min, int pulse_width_max)
{
    this->pulse_width_min = pulse_width_min;
    this->pulse_width_max = pulse_width_max;

    this->bytes = 1;
}
