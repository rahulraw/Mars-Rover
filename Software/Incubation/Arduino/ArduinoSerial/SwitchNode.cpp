#include "SwitchNode.h"

SwitchNode::SwitchNode(int pin)
{
    this->bytes = 1;
    this->pin = pin;
    this->value = 0;
}

char * SwitchNode::run()
{
    char data[2] = { 0, 1 };
    int new_value = digitalRead(this->pin);
    if (this->value != new_value && new_value == 1)
    {
        data[0] = (char) 1;
        data[1] = (char) new_value;
    }  
    this->value = new_value;
    return &data[0];
}
