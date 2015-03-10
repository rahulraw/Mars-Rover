#include "NodeHandler.h"

NodeHandler::NodeHandler()
{
    this->publisherLength = 0;
    this->subscriberLength = 0;
}

Subscriber * NodeHandler::getSubscriber(int topic)
{
    return this->subscribers[topic - 1];
}

Publisher * NodeHandler::getPublisher(int topic)
{
    return this->publishers[topic - 1];
}

void NodeHandler::addSubscriber(Subscriber * node)
{
    node->setTopicId(this->subscriberLength + 1);
    this->subscribers[this->subscriberLength] = node;
    this->subscriberLength++;
}

void NodeHandler::addPublisher(Publisher * node)
{
    node->setTopicId(this->publisherLength + 1);
    this->publishers[this->publisherLength] = node;
    this->publisherLength++;
}

bool NodeHandler:: validSubscriber(int topic)
{
    return topic <= this->subscriberLength;
}

bool NodeHandler:: validPublisher(int topic)
{
    return topic <= this->publisherLength;
}
