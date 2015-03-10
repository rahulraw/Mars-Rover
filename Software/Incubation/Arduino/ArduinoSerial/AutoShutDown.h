#ifndef AutoShutDown_h
#define AutoShutDown_h

#include "Arduino.h"
#include "Node.h"


class AutoShutDown : public Node
{
    public:
        int relay_pin;
        
        AutoShutDown(int relay_pin);

        virtual void run(char * data);
    private:
        void init(int relay_pin);
        void turn_off(int data_turn_off);
};

#endif
