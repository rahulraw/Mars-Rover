#include "NodeHandler.h"

NodeHandler::NodeHandler()
{
    this->length = 0;
}

Node * NodeHandler::getNode(int topic) 
{
    return this->nodes[topic - 1]; 
}

void NodeHandler::addNode(Node * node)
{
    node->setTopicId(this->length + 1);
    nodes[this->length] = node;    
    this->length++;
}

bool NodeHandler:: validNode(int topic)
{
    return topic <= this->length;
}
