#ifndef NodeHandler_h
#define NodeHandler_h

#include <Servo.h>
#include "Node.h"
#include "Subscriber.h"
#include "Publisher.h"

class NodeHandler
{
    public:
        NodeHandler();
        Subscriber * getSubscriber(int topic);
        bool validSubscriber(int topic);
        void addSubscriber(Subscriber * node);

        Publisher * getPublisher(int topic);
        bool validPublisher(int topic);
        void addPublisher(Publisher * node);
    private:
        Subscriber * subscribers[4];
        int subscriberLength;

        Publisher * publishers[4];
        int publisherLength;
};

#endif
