#ifndef CameraNode_h
#define CameraNode_h

#include <Servo.h>

#include "Arduino.h"
#include "Node.h"

class CameraNode : public Node
{
    public:
        ServoNode servoYaw;
        ServoNode servoPitch;

        CameraNode();
        CameraNode(int pulse_width_min1, int pulse_width_max1, int pulse_width_min2, int pulse_width_max2);

        virtual void run(char * data);
};

#endif
