#ifndef ServoNode_h
#define ServoNode_h

#include <Servo.h>

#include "Arduino.h"
#include "Node.h"
#include "Subscriber.h"

class ServoNode : public Subscriber
{
    public:
        int pulse_width_max;
        int pulse_width_min;
        Servo servo;

        ServoNode();
        ServoNode(int pulse_width_min, int pulse_width_max);

        virtual void run(char * data);

    private:
        void init(int pulse_width_min, int pulse_width_max);
        int getPulse(int angle);
};

#endif
