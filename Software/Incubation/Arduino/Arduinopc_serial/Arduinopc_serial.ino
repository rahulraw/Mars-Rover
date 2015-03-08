#include <Servo.h>

//Serial Initilization
int bytes[] = { 1, 2, 1 };
char data[2];
typedef void(*FunctionPointers)(char*);
FunctionPointers FunctionPointer[] = { &shutOff, &panAndTilt };//, &homing };

//LimitChecker
int limitA;


//AutoShutDown
int RelayPin;
boolean turn_off;

//Hall Effect Sensors(homing)
int SensorInput = 7;//Pin used for sensor FR
//int SensorInput = 8;//Pin used for sensor FL
//int SensorInput = 9;//Pin used for sensor BR
//int SensorInput = 10;//Pin used for sensor BL
int align_msg;

//CameraControl
//do a half half voltage divider w/ green=GND, red=+5 yellow=FB
#define PIN_CAM_ROTATE  4//backward/forward controlled PWM
#define CAM_ROTATE_STOP 1354
#define CAM_ROTATE_CW   CAM_ROTATE_STOP + 50
#define CAM_ROTATE_CCW  CAM_ROTATE_STOP - 50

#define PIN_CAM_PITCH   3//position controlled PWM
Servo srvCamRot, srvCamPitch;

void shutOff(char * data)
{
    if ((int) data[0])
    {
        digitalWrite(RelayPin, LOW);
    }
    else 
    {
        digitalWrite(RelayPin, HIGH);
    }
}

void panAndTilt(char * data)
{

}

//Camera functions
void camRotateCW(){
    srvCamRot.writeMicroseconds(CAM_ROTATE_CW);
}

void camRotateCCW(){
    srvCamRot.writeMicroseconds(CAM_ROTATE_CCW);
}

void camPitch(int angle){
    srvCamPitch.write(angle);
}
//Cameria Functions

void homing(char * data)
{
    digitalWrite(13,digitalRead(SensorInput));
    align_msg = (digitalRead(SensorInput));//Sends 1 if aligned 0 if not.
    align_msg = 1;
    delay(80);
}


void setup() {

    Serial.begin(9600);
    attachInterrupt(0, limitCheck, CHANGE); // Attaching Int.0 = pin 2 -- Limit Switch
    typedef void(*FunctionPointer)(char*);
    FunctionPointer FunctionPointers[] = { &shutOff, &panAndTilt, &homing };

    //autoshutdown
    pinMode(RelayPin, OUTPUT);  

    //Homing
    //SensorInit(SensorInputFR,SensorOutputFR);
    //SensorInit(SensorInputFL,SensorOutputFL);
    //SensorInit(SensorInputBR,SensorOutputBR);
    //SensorInit(SensorInputBL,SensorOutputBL);
    pinMode(SensorInput, INPUT);
    pinMode(13, OUTPUT);
    


    //CameraControl
    srvCamRot.attach(PIN_CAM_ROTATE);
    srvCamRot.writeMicroseconds(CAM_ROTATE_STOP);
    camRotateCCW();
    srvCamPitch.attach(PIN_CAM_PITCH);
    camPitch(125);

    //char data[2];
}

void loop() {

    if (Serial.available() > 0) {

        int topic = (int) Serial.read();

        for (int i = 0; i < bytes[topic]; i++) {
            data[i] = Serial.read();
        }

        handleSubscribers(topic, data);
    }
    delay(20);

}

void limitCheck()

{
    //delay(2);
    int newValue = digitalRead(2);
     
    if (limitA != newValue && newValue == 1){
        Serial.print("New Value: ");
        Serial.print(newValue);
        Serial.print(" limitA: ");
        Serial.println(limitA);      
    }  

    limitA = newValue;
}



void handleSubscribers(int topic, char * data) {
    FunctionPointer[topic](data);
}

void handlePubishers() {
        
}


