#ifndef NodeHandler_h
#define NodeHandler_h

#define ACCEPT_SERIAL 255
#define SEND_SERIAL 254

#include <Servo.h>
#include "Arduino.h"
#include "Node.h"
#include "Subscriber.h"
#include "Publisher.h"

class NodeHandler
{
    public:
        NodeHandler();
        NodeHandler(int delay, int timeout);

        Subscriber * getSubscriber(int topic);
        bool validSubscriber(int topic);
        void addSubscriber(Subscriber * node);

        Publisher * getPublisher(int topic);
        bool validPublisher(int topic);
        void addPublisher(Publisher * node);

        int subscriberLength;
        int publisherLength;

        void run();
    private:
        Subscriber * subscribers[4];

        Publisher * publishers[4];

        int topic;
        int bytes;
        int publish_delay;
        int timeout;
        unsigned long currentTime;

        char data[5];

        void handleSubscriber();
        void handlePublishers();
};

#endif
