#ifndef Publisher_h
#define Publisher_h

#include "Node.h"

class Publisher : public Node
{
    public:
        virtual ~Publisher() {}

        virtual char * run() = 0;
};

#endif
