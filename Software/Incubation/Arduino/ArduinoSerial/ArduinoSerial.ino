#include <math.h>
#include <Servo.h>

#include "Node.h"
#include "NodeHandler.h"
#include "HomeNode.h"
#include "AutoShutDown.h"
#include "CameraNode.h"
#include "Manipulator.h"

#define SERVO_MANIPULATOR_CLAW      6
#define SERVO_MANIPULATOR_SWING     11
#define SERVO_MANIPULATOR_ROTATE    12

#define SERVO_CAM_ROTATE    2
#define SERVO_CAM_PITCH     3

#define PULSE_WIDTH_MAX_CLAW 1400
#define PULSE_WIDTH_MIN_CLAW 900

#define PULSE_WIDTH_MAX 2000
#define PULSE_WIDTH_MIN 1000

#define RELAY_PIN 49

#define HOME_BACK_RIGHT 50
#define HOME_FRONT_RIGHT 51
#define HOME_BACK_LEFT 52
#define HOME_FRONT_LEFT 53

NodeHandler nodeHandler(20, 2000);
ServoNode claw(PULSE_WIDTH_MIN_CLAW, PULSE_WIDTH_MAX_CLAW);
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

    claw.servo.attach(SERVO_MANIPULATOR_CLAW, claw.pulse_width_min, claw.pulse_width_max);

    cameraNode.servoRotate->servo.attach(SERVO_CAM_ROTATE);
    cameraNode.servoPitch->servo.attach(SERVO_CAM_PITCH);
    cameraNode.servoRotate->servo.writeMicroseconds(CAM_ROTATE_STOP);
    cameraNode.servoPitch->servo.write(CAM_PITCH_STRAIGHT);

    nodeHandler.addSubscriber(&claw);
    nodeHandler.addSubscriber(&autoShutDown);
    nodeHandler.addSubscriber(&cameraNode);

    nodeHandler.addPublisher(&homeNode);
}

void loop()
{
    nodeHandler.run();
}
