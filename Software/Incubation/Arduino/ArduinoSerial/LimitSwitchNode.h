#ifndef LimitSwitchNode_h
#define LimitSwitchNode_h

#include <Servo.h>

#include "Arduino.h"
#include "Node.h"
#include "ServoNode.h"
#include "Publisher.h"

class LimitSwitchNode : Publisher
{
    public:
        LimitSwitchNode();
        virtual char * run();

    private:
        int limitSwitch;
};

#endif
