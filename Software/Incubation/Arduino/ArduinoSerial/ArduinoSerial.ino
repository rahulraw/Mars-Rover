#include <math.h>
#include <Servo.h>

#include "ServoNode.h"
#include "Node.h"
#include "NodeHandler.h"
#include "HomeNode.h"
#include "AutoShutDown.h"

#define CLAW_PIN 10

#define PULSE_WIDTH_MAX 1400
#define PULSE_WIDTH_MIN 900
#define RELAY_PIN 49

#define HOME_BACK_RIGHT 50
#define HOME_FRONT_RIGHT 51
#define HOME_BACK_LEFT 52
#define HOME_FRONT_LEFT 53

NodeHandler nodeHandler(20, 2000);
ServoNode claw(PULSE_WIDTH_MIN, PULSE_WIDTH_MAX);
CameraNode cameraNode(PULSE_WIDTH_MIN, PULSE_WIDTH_MAX, PULSE_WIDTH_MIN, PULSE_WIDTH_MAX);
AutoShutDown autoShutDown(RELAY_PIN);
HomeNode homeNode(HOME_FRONT_LEFT, HOME_FRONT_RIGHT, HOME_BACK_LEFT, HOME_BACK_RIGHT);

void setup()
{
    Serial.begin(9600);
    pinMode(RELAY_PIN, OUTPUT);

    pinMode(HOME_BACK_RIGHT, INPUT_PULLUP);
    pinMode(HOME_FRONT_RIGHT, INPUT_PULLUP);
    pinMode(HOME_BACK_LEFT, INPUT_PULLUP);
    pinMode(HOME_FRONT_LEFT, INPUT_PULLUP);

    claw.claw.attach(CLAW_PIN, claw.pulse_width_min, claw.pulse_width_max);
    nodeHandler.addSubscriber(&claw);
    nodeHandler.addSubscriber(&autoShutDown);
    nodeHandler.addSubscriber(&cameraNode);

    nodeHandler.addPublisher(&homeNode);
}

void loop()
{
    nodeHandler.run();
}
