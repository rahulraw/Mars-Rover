#include <math.h>

#include "Arduino.h"
#include "Claw.h"

Claw::Claw()
{
    this->init(PULSE_WIDTH_MIN, PULSE_WIDTH_MAX);
}

Claw::Claw(int pulse_width_min, int pulse_width_max)
{
    this->init(pulse_width_min, pulse_width_max);
}

void Claw::run(char * data)
{
    int pulse = this->getPulse((int) data[0]);
    this->claw.writeMicroseconds(pulse);
}

int Claw::getPulse(int angle)
{
   return (int) floor(((this->pulse_width_max - this->pulse_width_min) / 90.0) * angle) + this->pulse_width_min;
}

void Claw::init(int pulse_width_min, int pulse_width_max)
{
    this->pulse_width_min = pulse_width_min;
    this->pulse_width_max = pulse_width_max;

    this->bytes = 1;
}
