#ifndef Node_h
#define Node_h

class Node
{
    public:
        virtual ~Node() {}

        int getTopicId();
        int getBytes();

        void setTopicId(int topicId);
    protected:
        int topicId;
        int bytes;
};

#endif
