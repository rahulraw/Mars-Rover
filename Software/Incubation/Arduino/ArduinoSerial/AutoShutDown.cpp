#include "Arduino.h"
#include "AutoShutDown.h"


AutoShutDown::AutoShutDown(int relay_pin)
{
    this->init(relay_pin);
}

void AutoShutDown::init(int relay_pin)
{
    this->relay_pin = relay_pin;
    digitalWrite(this->relay_pin, HIGH);
    this->bytes = 1;
}

void AutoShutDown::run(char * data)
{
   this->turn_off((int) data[0]); 
}

void AutoShutDown::turn_off(int data_turn_off)
{
    bool type = data_turn_off == 1? LOW : HIGH;
    digitalWrite(this->relay_pin, type);
}
