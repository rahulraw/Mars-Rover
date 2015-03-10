#include <math.h>
#include <Servo.h>

#include "ServoNode.h"
#include "Node.h"
#include "NodeHandler.h"

#define CLAW_PIN 10

#define PULSE_WIDTH_MAX 1340
#define PULSE_WIDTH_MIN 970

NodeHandler nodeHandler(20, 500);
ServoNode claw(PULSE_WIDTH_MIN, PULSE_WIDTH_MAX);
ServoNode cameraYaw(PULSE_WIDTH_MIN, PULSE_WIDTH_MAX);
ServoNode cameraPitch(PULSE_WIDTH_MIN, PULSE_WIDTH_MAX);

void setup()
{
    Serial.begin(9600);
    claw.claw.attach(CLAW_PIN, claw.pulse_width_min, claw.pulse_width_max);
    nodeHandler.addSubscriber(&claw);
    nodeHandler.addSubscriber(&cameraYaw);
    nodeHandler.addSubscriber(&cameraPitch);
}

void loop()
{
    nodeHandler.run();
}
