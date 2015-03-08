#include <Servo.h>

class Arduino-PC
{
    int bytes[] = { 1, 2, 1 };
    char data[2];
    typedef void(*FunctionPointers)(char*) = { &shutOff, &panAndTilt, &homing };//probably doesn't work.

    int limitA;

    int align_msg;

    int relayPin, sensorInput;
    unsigned const PIN_CAM_ROTATE, CAM_ROTATE_STOP, CAM_ROTATE_CW, CAM_ROTATE_CCW, PIN_CAM_PITCH; 
   
    Servo srvCamRot, srvCamPitch;
    

    public:
        void set_values(int, int, int, int, int, int, int);
            
        void handleSubscribers(int topic, char * data)
        {
            FunctionPointer[topic](data);
        }

}


