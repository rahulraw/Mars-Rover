//Position PID example.  Velocity PID is not used. Speeds are not controlled by feedback.
//This example is the most like a standard RC servo

//Warning!!!!
//Arduino Mega and Leonardo chips only support some pins for receiving data back from the RoboClaw
//This is because only some pins of these boards support PCINT interrupts or are UART receivers.
//Mega: 0,10,11,12,13,14,15,17,19,50,51,52,53,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15
//Leonardo: 0,8,9,10,11

//Arduino Due currently does not support software serial. Only hardware uarts can be used, pins 0/1, 14/15, 16/17 or 18/19.

//Includes required to use Roboclaw library
#include "BMSerial.h"
#include "RoboClaw.h"

//Roboclaw Address
#define address 0x80

//Note: PID coeffcients will need to be tuned for this code to work with your motor.

//Velocity PID coefficients
#define Kp 1000
#define Ki 2000
#define Kd 3000
#define Qpps 3601

//Position PID coefficients
#define PosKp 2000
#define PosKi 4
#define PosKd 20000
#define KiMax 12.5
#define DeadZone 10
#define Min 50
#define Max 1950


//Setup communcaitions with roboclaw. Use pins 10 and 11 with 10ms timeout
RoboClaw roboclaw(15,14,10000);


void setup() {
  //Open terminal and roboclaw serial ports
  roboclaw.begin(38400);
  
  
  //Use Absolute Encoder and Enable encoders in RC mode
  roboclaw.SetM1EncoderMode(address,0x81);

  //Set PID Coefficients
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,Qpps);

  //Save settings in Roboclaw eeprom.  Uncomment to save settings.
  roboclaw.WriteNVM(address);
}

//Main loop
void loop() {

}
