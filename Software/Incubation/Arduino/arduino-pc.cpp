#include "arduino-pc.h"

void Arduino-PC::set_values(int a, int b, int c, int d, int e, int f, int g) 
{
    relayPin = a;
    sensorInput = b;
    PIN_CAM_ROTATE = c; 
    CAM_ROTATE_STOP = d;
    CAM_ROTATE_CW = e;
    CAM_ROTATE_CCW = f;
    PIN_CAM_PITCH = g;

}

