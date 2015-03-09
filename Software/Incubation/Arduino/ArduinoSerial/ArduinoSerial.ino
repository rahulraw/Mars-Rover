#include <math.h>
#include <Servo.h>

#include "Claw.h"
#include "Node.h"
#include "NodeHandler.h"

#define DELAY 50
#define CLAW_PIN 10

char data[2];
int topic;
int bytes;
Node * node;

NodeHandler nodeHandler;
Claw claw;

void setup()
{
    Serial.begin(9600);
    claw.claw.attach(CLAW_PIN);
    nodeHandler.addNode(&claw);
}

void loop()
{
    if (Serial.available()) 
    {
        topic = Serial.read();
        if (nodeHandler.validNode(topic)) {
            node = nodeHandler.getNode(topic);
            bytes = node->getBytes();

            for (int i = 0; i < bytes; i++)
            {
                while (!Serial.available());
                data[i] = (int) Serial.read();
            }
            node->run(data);
        }
    }
    delay(DELAY);
}

