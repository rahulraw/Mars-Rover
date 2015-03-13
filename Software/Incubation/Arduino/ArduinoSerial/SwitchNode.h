#ifndef SwitchNode_h
#define SwitchNode_h

#include <Servo.h>

#include "Arduino.h"
#include "Node.h"
#include "Publisher.h"

class SwitchNode : Publisher
{
    public:
        SwitchNode();
        SwitchNode(int pin);
        virtual char * run();

    private:
        int pin;
        int value;
};

#endif
