#ifndef NodeHandler_h
#define NodeHandler_h

#include <Servo.h>
#include "Node.h"

class NodeHandler
{
    public:
        NodeHandler();
        Node * getNode(int topic);
        void addNode(Node * node);
        bool validNode(int topic);
    private:
        Node * nodes[4];
        int length;
};

#endif
