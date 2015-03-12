#include "HomeNode.h"

HomeNode::HomeNode(int pin1, int pin2, int pin3, int pin4)
{
    this->bytes = 4;

    this->pins[0] = pin1;
    this->pins[1] = pin2;
    this->pins[2] = pin3;
    this->pins[3] = pin4;
}

char * HomeNode::run()
{
    this->values[0] = (char) 0;
    for (int i = 0; i < this->bytes; i++)
    {
        int new_value = 0;
        new_value = digitalRead(this->pins[i]);
        if (new_value == this->values[i + 1])
        {
            this->values[0] = (char) 1;
            this->values[i + 1] = (char) (1 - new_value);
        }  
    }
    return &(this->values[0]);
}
