#include "NodeHandler.h"

NodeHandler::NodeHandler()
{
    this->publisherLength = 0;
    this->subscriberLength = 0;
    this->currentTime = millis();
    this->publish_delay = 20;
    this->timeout = 500;
}

NodeHandler::NodeHandler(int publish_delay, int timeout)
{
    this->publisherLength = 0;
    this->subscriberLength = 0;
    this->currentTime = millis();
    this->publish_delay = publish_delay;
    this->timeout = timeout;
}

Subscriber * NodeHandler::getSubscriber(int topic)
{
    return this->subscribers[topic - 1];
}

Publisher * NodeHandler::getPublisher(int topic)
{
    return this->publishers[topic - 1];
}

void NodeHandler::run()
{
    Serial.write(ACCEPT_SERIAL);
    while (!Serial.available() && millis() - this->currentTime < this->timeout)
    {
        this->handlePublishers();
        delay(this->publish_delay);
    }

    this->currentTime = millis();

    if (Serial.available())
    {
        this->handleSubscriber();
    }
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

void NodeHandler::handlePublishers()
{
    for (int topic = 0; topic < this->publisherLength; topic++)
    {
        this->bytes = this->getPublisher(topic)->getBytes();
        char * pub_data = this->getPublisher(topic)->run();

        // The first byte that publisher run has to send back is whether
        // it wants to send a message or not
        if (pub_data[0] == 1)
        {
            Serial.write(topic);
            Serial.write(this->bytes);

            for (int i = 0; i < this->bytes; i++) {
                Serial.write(pub_data[i]);
            }
        }
    }
}

void NodeHandler::handleSubscriber()
{
    int topic = Serial.read();
    if (this->validSubscriber(topic))
    {
        this->bytes = this->getSubscriber(topic)->getBytes();

        for (int i = 0; i < bytes; i++)
        {
            this->data[i] = (int) Serial.read();
        }
        this->getSubscriber(topic)->run(this->data);
    }
}
