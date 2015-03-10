#ifndef Subscriber_h
#define Subscriber_h

#include "Node.h"

class Subscriber : public Node
{
    public:
        virtual ~Subscriber() {}

        virtual void run(char * data) = 0;
};

#endif
