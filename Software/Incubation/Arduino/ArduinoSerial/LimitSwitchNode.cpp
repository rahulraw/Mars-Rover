#include "LimitSwitchNode.h"

LimitSwitchNode::LimitSwitchNode()
{
    this->bytes = 1;
}

char * LimitSwitchNode::run()
{
    char data[2] = { 0, 1 };
    int newValue = digitalRead(2);
    if (this->limitSwitch != newValue && newValue == 1)
    {
        data[0] = (char) 1;
        data[1] = (char) this->limitSwitch;
    }  
    this->limitSwitch = newValue;
    return &data[0];
}
