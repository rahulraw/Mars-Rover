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
    this->handlePublishers();
    this->handleSubscriber();
    delay(this->publish_delay);
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
    return topic <= this->subscriberLength && topic > 0;
}

bool NodeHandler:: validPublisher(int topic)
{
    return topic <= this->publisherLength && topic > 0;
}

void NodeHandler::handlePublishers()
{
    for (int topic = 1; topic < this->publisherLength + 1; topic++)
    {
        int bytes = this->getPublisher(topic)->getBytes();
        char * pub_data = this->getPublisher(topic)->run();

        // The first byte that publisher run has to send back is whether
        // it wants to send a message or not
        if (pub_data[0] == 1)
        {
            Serial.write(SEND_SERIAL);
            Serial.write(topic);
            Serial.write(bytes);

            for (int i = 0; i < bytes; i++)
            {
                Serial.write(pub_data[i + 1]);
            }
        }
    }
}

void NodeHandler::handleSubscriber()
{
    Serial.write(ACCEPT_SERIAL);
    if (Serial.available())
    {
        int numTopics = Serial.read();
        for (int j = 0; j < numTopics; j++)
        {
            int topic = Serial.read();
            if (this->validSubscriber(topic))
            {
                for (int i = 0; i < this->getSubscriber(topic)->getBytes(); i++)
                {
                    this->data[i] = (int) Serial.read();
                }
                this->getSubscriber(topic)->run(this->data);
            }
        }
    }
}
