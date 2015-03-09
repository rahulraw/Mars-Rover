#ifndef Claw_h
#define Claw_h

#include <Servo.h>

#include "Arduino.h"
#include "Node.h"

#define PULSE_WIDTH_MAX 1340
#define PULSE_WIDTH_MIN 970

class Claw : public Node
{
    public:
        int pulse_width_max;
        int pulse_width_min;
        Servo claw;

        Claw();
        Claw(int pulse_width_min, int pulse_width_max);

        virtual void run(char * data);
    private:
        void init(int pulse_width_min, int pulse_width_max);
        int getPulse(int angle);
};

#endif
