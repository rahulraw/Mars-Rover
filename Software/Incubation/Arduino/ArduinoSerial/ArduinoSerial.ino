#include <math.h>
#include <Servo.h>

#include "ServoNode.h"
#include "Node.h"
#include "NodeHandler.h"
#include "AutoShutDown.h"

#define CLAW_PIN 10

#define PULSE_WIDTH_MAX 1400
#define PULSE_WIDTH_MIN 900
#define RELAY_PIN 13

NodeHandler nodeHandler(20, 2000);
ServoNode claw(PULSE_WIDTH_MIN, PULSE_WIDTH_MAX);
ServoNode cameraYaw(PULSE_WIDTH_MIN, PULSE_WIDTH_MAX);
ServoNode cameraPitch(PULSE_WIDTH_MIN, PULSE_WIDTH_MAX);
AutoShutDown autoShutDown(RELAY_PIN);

void setup()
{
    Serial.begin(9600);
    pinMode(RELAY_PIN, OUTPUT);
    claw.claw.attach(CLAW_PIN, claw.pulse_width_min, claw.pulse_width_max);
    nodeHandler.addSubscriber(&claw);
    nodeHandler.addSubscriber(&autoShutDown);
    nodeHandler.addSubscriber(&cameraYaw);
    nodeHandler.addSubscriber(&cameraPitch);
}

void loop()
{
    nodeHandler.run();
}
