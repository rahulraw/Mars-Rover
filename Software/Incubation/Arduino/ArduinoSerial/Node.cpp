#include <math.h>
#include "Node.h"

int Node::getTopicId()
{
    return this->topicId;
}

int Node::getBytes()
{
    return this->bytes;
}

void Node::setTopicId(int topicId)
{
    this->topicId = topicId;
}
