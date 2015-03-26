#ifndef HomeNode_h
#define HomeNode_h

#include <Servo.h>

#include "Arduino.h"
#include "Node.h"
#include "SwitchNode.h"
#include "Publisher.h"

class HomeNode : public Publisher
{
    public:
        HomeNode();
        HomeNode(int pin1, int pin2, int pin3, int pin4);
        virtual char * run();

    private:
        int pins[4];
        char values[5] = { 0, 0, 0, 0, 0 };
};

#endif
