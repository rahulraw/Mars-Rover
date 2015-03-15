#ifndef CameraNode_h
#define CameraNode_h

#include <Servo.h>

#include "Arduino.h"
#include "Node.h"
#include "ServoNode.h"
#include "Subscriber.h"

#define CAM_ROTATE_STOP 1362
#define CAM_ROTATE_CW   CAM_ROTATE_STOP + 60
#define CAM_ROTATE_CCW  CAM_ROTATE_STOP - 60

#define CAM_PITCH_LOW_LIMIT 140
#define CAM_PITCH_STRAIGHT 140
#define CAM_PITCH_HIGH_LIMIT 160

#define COMMAND_STOP    1
#define COMMAND_LOW     0
#define COMMAND_HIGH    2

#define PIN_CAM_ZOOM    8

class CameraNode : public Subscriber
{
    public:
        CameraNode();
        ~CameraNode();
        CameraNode(int pulse_width_min1, int pulse_width_max1, int pulse_width_min2, int pulse_width_max2);

        ServoNode * servoRotate;
        ServoNode * servoPitch;

        virtual void run(char * data);
};

#endif
