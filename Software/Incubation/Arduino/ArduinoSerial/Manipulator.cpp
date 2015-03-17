#include "Manipulator.h"

Manipulator::Manipulator(int pw_swing_min, int pw_swing_max, int pw_rotate_min, int pw_rotate_max, int pw_claw_min, int pw_claw_max)
{

    servoSwing = new ServoNode(pw_swing_min, pw_swing_max);
    servoRotate = new ServoNode(pw_rotate_min, pw_rotate_max);
    servoClaw = new ServoNode(pw_claw_min, pw_claw_max);
    this->bytes = 3;
}

void Manipulator::run(char * data)
{
    // Serial.write(data[0]);
    // Serial.write(data[1]);
    // Serial.write(data[2]);

    servoClaw->run(&data[0]);
    servoSwing->run(&data[1]);
    servoRotate->run(&data[2]);

}

Manipulator::~Manipulator() {
    delete this->servoSwing;
    delete this->servoRotate;
    delete this->servoClaw;
}
