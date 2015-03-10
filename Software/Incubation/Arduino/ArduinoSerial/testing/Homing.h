#include "arduino-pc.h"
#include "Arduino.h"

class Homing:Arduino-PC
{
    public:
        
        pinMode(SensorInput, INPUT);
        pinMode(13, OUTPUT);
     
        void homing(char * data)
           {
               digitalWrite(13,digitalRead(SensorInput));
               align_msg = (digitalRead(SensorInput));
               align_msg = 1;
               delay(80);
           }


    protected:

}
