#ifndef CameraNode_h
#define CameraNode_h

#include <Servo.h>

#include "Arduino.h"
#include "Node.h"
#include "ServoNode.h"
#include "Subscriber.h"

class CameraNode : public Subscriber
{
    public:
        CameraNode();
        ~CameraNode();
        CameraNode(int pulse_width_min1, int pulse_width_max1, int pulse_width_min2, int pulse_width_max2);

        ServoNode * servoYaw;
        ServoNode * servoPitch;

        virtual void run(char * data);
};

#endif
