#include "arduino-pc.h"

class ShutOff:Arduino-PC
{
    public:
        void shutOff(char * data)
        {
            if ((int) data[0])
            { 
                digitalWrite(RelayPin, LOW);
            }
            else
            {
                digitalWrite(RelayPin, HIGH);
            }
        }
    
    protected:
}


