#include <math.h>
#include <Servo.h>

#include "ServoNode.h"
#include "Node.h"
#include "NodeHandler.h"

#define DELAY 50
#define CLAW_PIN 10

#define PULSE_WIDTH_MAX 1340
#define PULSE_WIDTH_MIN 970

char data[2];
int topic;
int bytes;
Node * node;

NodeHandler nodeHandler;
ServoNode claw(PULSE_WIDTH_MIN, PULSE_WIDTH_MAX);
ServoNode cameraYaw(PULSE_WIDTH_MIN, PULSE_WIDTH_MAX);
ServoNode cameraPitch(PULSE_WIDTH_MIN, PULSE_WIDTH_MAX);

void setup()
{
    Serial.begin(9600);
    claw.claw.attach(CLAW_PIN);
    nodeHandler.addNode(&claw);
    nodeHandler.addNode(&cameraYaw);
    nodeHandler.addNode(&cameraPitch);
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

