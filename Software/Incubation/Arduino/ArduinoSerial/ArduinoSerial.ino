#include <math.h>
#include <Servo.h>

#include "Claw.h"
#include "Node.h"
#include "NodeHandler.h"

#define TIMEOUT 500
#define ACCEPT_SERIAL 255
#define DELAY 50
#define CLAW_PIN 10

char data[2];
int topic;
int bytes;
Node * node;

unsigned long currentTime;

NodeHandler nodeHandler;
Claw claw;

void setup()
{
    Serial.begin(9600);
    claw.claw.attach(CLAW_PIN, claw.pulse_width_min, claw.pulse_width_max);
    nodeHandler.addNode(&claw);
}

void loop()
{
    if (waitForMessage())
    {
        topic = Serial.read();
        if (nodeHandler.validNode(topic)) {
            node = nodeHandler.getNode(topic);
            bytes = node->getBytes();

            for (int i = 0; i < bytes; i++)
            {
                data[i] = (int) Serial.read();
            }
            node->run(data);
        }
    }
    delay(DELAY);
}

bool waitForMessage()
{
    Serial.write(ACCEPT_SERIAL);
    while (!Serial.available() || millis() - currentTime < TIMEOUT);
    currentTime = millis();
    return Serial.available();
}
