#include "arduino-pc.h"

class LimitChecker:Arduino-PC
{
    public:
        void Limitcheck()
        {
            //delay(2);
            int newValue = digitalRead(2);
            if (limitA != newValue && newValue == 1)
            {
                Serial.print("New Value: ");
                Serial.print(newValue);
                Serial.print(" limitA: ");
                Serial.println(limitA);      
            }  

            limitA = newValue;
        }
    protected:
        int newValue
}
