#ifndef Manipulator_h
#define Manipulator_h

#include <Servo.h>

#include "Arduino.h"
#include "Node.h"
#include "ServoNode.h"
#include "Subscriber.h"

class Manipulator : public Subscriber
{
    public:
        Manipulator();
        ~Manipulator();
        Manipulator(int pw_swing_min, int pw_swing_max, int pw_rotate_min, int pw_rotate_max, int pw_claw_min, int pw_claw_max);

        ServoNode * servoRotate;
        ServoNode * servoSwing;
        ServoNode * servoClaw;

        virtual void run(char * data);
};

#endif
