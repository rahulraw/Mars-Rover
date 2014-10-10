#include "BMSerial.h"
#include "RoboClaw.h"

#define CHANNEL_1_PIN 2 //Throttle
#define CHANNEL_2_PIN 3 //Aile
#define CHANNEL_3_PIN 21 //Elev
#define CHANNEL_4_PIN 20 //Rudd
#define CHANNEL_5_PIN 18 //Gear Switch

#define SWITCH_THRESHOLD	1600
#define SWITCH_THRESHOLD_AILE	1200

volatile unsigned long timerStartThro;
volatile unsigned long timerStartAile;
volatile unsigned long timerStartElev;
volatile unsigned long timerStartRudd;
volatile unsigned long timerStartGear;

volatile int pulseTimeThro = 1512; 
volatile int pulseTimeAile;
volatile int pulseTimeElev; 
volatile int pulseTimeRudd = 1308; // zero velocity command
volatile int pulseTimeGear = SWITCH_THRESHOLD + 50;

volatile int last_interrupt_time_1;
volatile int last_interrupt_time_2;
volatile int last_interrupt_time_3;
volatile int last_interrupt_time_4;

volatile int check_Thro;
volatile int check_Aile;
volatile int check_Elev;
volatile int check_Rudd;

int gearSwitchBuffer = 5;
int check_Gear[] = {0, 100, 200, 300, 400};

const int max_stick_change=200;
const int stick_noise = 10;

const int switch_noise = 50;

const int encThresh = 200;

int i=0;

int direct;
int spd;



void calcThro()
{
  //last_interrupt_time_1 = micros();

  if(digitalRead(CHANNEL_1_PIN) == HIGH)
  {
    timerStartThro = micros();
  }

  else
  {
    if(timerStartThro > 0)
    {
      check_Thro = (micros() - timerStartThro);
      if (check_Thro > 900 && check_Thro < 2150 && abs(check_Thro - pulseTimeThro) < max_stick_change){
        pulseTimeThro = check_Thro;
      }
      timerStartThro = 0;
      
    }
  }
}

void calcAile()
{
  //last_interrupt_time_2 = micros();

  if(digitalRead(CHANNEL_2_PIN) == HIGH)
  {
    timerStartAile = micros();
  }

  else
  {
    if(timerStartAile > 0)
    {
      check_Aile = (micros() - timerStartAile);
      if (check_Aile > 900 && check_Aile < 2150 && abs(check_Aile - pulseTimeAile) < max_stick_change){
        pulseTimeAile = check_Aile;
      }
      timerStartAile = 0;
    }
  }
}


void calcElev()
{
  //last_interrupt_time_3 = micros();

  if(digitalRead(CHANNEL_3_PIN) == HIGH)
  {
    timerStartElev = micros();
  }

  else
  {
    if(timerStartElev > 0)
    {
      check_Elev = (micros() - timerStartElev);
      if (check_Elev > 900 && check_Elev < 2150 && abs(check_Elev - pulseTimeElev) < max_stick_change){
        pulseTimeElev = check_Elev;
      }
      timerStartElev = 0;
    }
  }
}


void calcRudd()
{
  //last_interrupt_time_4 = micros();
  
  if(digitalRead(CHANNEL_4_PIN) == HIGH)
  {
    timerStartRudd = micros();
  }

  else
  {
    if(timerStartRudd > 0)
    {
      check_Rudd = (micros() - timerStartRudd);
      if (check_Rudd > 900 && check_Rudd < 2150 && abs(check_Rudd - pulseTimeRudd) < max_stick_change){
        pulseTimeRudd = check_Rudd;
      }
      timerStartRudd = 0;
	}
  }
}

void calcGear()
{

  if (digitalRead(CHANNEL_5_PIN) == HIGH) {
    timerStartGear = micros();
  }

  else
  {
    if(timerStartGear > 0)
    {
      check_Gear[i] = (micros() - timerStartGear);
      if (abs(check_Gear[i] - check_Gear[0]) < switch_noise && abs(check_Gear[i] - check_Gear[1]) < switch_noise && abs(check_Gear[i] - check_Gear[2]) < switch_noise && abs(check_Gear[i] - check_Gear[3]) < switch_noise && abs(check_Gear[i] - check_Gear[4]) < switch_noise){
        pulseTimeGear = check_Gear[i];
      }
      timerStartGear = 0;
	  i++;
	  if(i>=gearSwitchBuffer){
		i=0;
	  }
	}
  }
}



double input, DEG, encmod1, encmod2, y;


//Roboclaw Address
#define address 0x80

//Setup communcaitions with roboclaw. Use pins 10 and 11 with 10ms timeout
RoboClaw roboclaw1(15,14,10000); //front
RoboClaw roboclaw2(17,16,10000); //back

void setup() {
  //Open roboclaw serial ports
  
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  //timer_start = 0;
  attachInterrupt(0, calcThro, CHANGE);
  attachInterrupt(1, calcAile, CHANGE);
  attachInterrupt(2, calcElev, CHANGE);
  attachInterrupt(3, calcRudd, CHANGE);
  attachInterrupt(5, calcGear, CHANGE);
  
  Serial.begin(9600);
  
  Serial.println("Waiting.......");
  //do the switch flick thing to preceed with controlling the robot
  
  while (pulseTimeGear > SWITCH_THRESHOLD);
  while (pulseTimeGear < SWITCH_THRESHOLD);
  
  roboclaw1.begin(38400);
  roboclaw2.begin(38400);
  
  digitalWrite(13, HIGH);

  Serial.println("Start!");
}

void loop() {
  
// reading in from the remote control  
  if(pulseTimeThro <= 1033) // Direction
    pulseTimeThro = 1033;
  else if(pulseTimeThro >= 1992)
    pulseTimeThro = 1992;
    
  direct = map(pulseTimeThro, 1033, 1992, 90, -90); // rc direction stick position in degrees
  
  //Serial.println(pulseTimeRudd);
  if(pulseTimeRudd <= 1009)
    pulseTimeRudd = 1009;
  else if(pulseTimeRudd >= 1607)
    pulseTimeRudd = 1607;
  spd = map(pulseTimeRudd, 1009, 1607, -100, 100); // converst rc stick speed to speed values (-127 to 127)

// read in encoder value   
  
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;
  uint32_t enc1= roboclaw1.ReadEncM1(address, &status1, &valid1);
  uint32_t enc2= roboclaw2.ReadEncM1(address, &status2, &valid2);
  //uint32_t enc3= roboclaw3.ReadEncM1(address, &status3, &valid3);
  
  DEG = 70.368*direct; // calculate desired change in degrees in ticks
  y= 2147483648 + DEG; // y is desired position in ticks

  encmod1=enc1 + 2147483648; // shift encoder value by large number (to avoid crossing 0)
  encmod2=enc2 + 2147483648;
  //encmod3=enc3 + 2147483648;
  
  // controlling speed of wheel
  
  
  if (pulseTimeGear > SWITCH_THRESHOLD || pulseTimeGear < SWITCH_THRESHOLD_AILE){
  	stopAllMotion();
  }
  else{

	if (spd > 5)
	{
		roboclaw1.BackwardM2(address, spd);
		roboclaw2.BackwardM2(address, spd); // oposite because motors facing different directions
		//roboclaw3.ForwardM2 (address, spd);
	}

	else if (spd < -5)

	{
		roboclaw1.ForwardM2(address, -spd);
		roboclaw2.ForwardM2(address, -spd);
		//roboclaw3.BackwardM2 (address, -spd);

	}

	else {
		roboclaw1.ForwardM2(address, 0);
		roboclaw2.ForwardM2(address, 0);
		//   roboclaw3.ForwardM2 (address, 0);
	}


	//Bottom Left Wheel

	if (y - encThresh > encmod1) // going left/right

	{
		roboclaw1.ForwardM1(address, 20);

	}


	else if (y + encThresh < encmod1) // going right/left

	{
		roboclaw1.BackwardM1(address, 20);
	}

	else
	{
		roboclaw1.BackwardM1(address, 0);
	}

	//Bottom Right Wheel

	if (y - encThresh > encmod2) // going left/right

	{
		roboclaw2.ForwardM1(address, 20);
	}


	else if (y + encThresh < encmod2) // going right/left

	{
		roboclaw2.BackwardM1(address, 20);
	}

	else
	{

		roboclaw2.BackwardM1(address, 0);
	}

	/*

	//Front Left Wheel

	if (y -  500 > encmod3 ) // going left/right

	{
	roboclaw3.ForwardM1 (address, 20);
	}


	else if (y + 500 < encmod3) // going right/left

	{
	roboclaw3.BackwardM1(address,20);
	}

	else
	{

	roboclaw3.BackwardM1(address,0);
	}
	*/
  }
  Serial.print(pulseTimeThro);
  Serial.print("\t");
  Serial.print(pulseTimeAile);
  Serial.print("\t");
  Serial.print(pulseTimeElev);
  Serial.print("\t");
  Serial.print(pulseTimeRudd);
  Serial.print("\t");
  Serial.print(pulseTimeGear);
  Serial.print("\t");
  Serial.print(direct);
  Serial.print("\t");
  Serial.print(spd);
  Serial.print("\t");
  Serial.print(DEG);
  Serial.print("\t");

  Serial.print(enc1);
  
  Serial.print ("\t");
  Serial.print (encmod1);
  Serial.print ("\t");
  Serial.print ((encmod1-2147483648)/70.637);
  Serial.print ("\t");
  Serial.println (y);

}

void stopAllMotion(){
	roboclaw1.ForwardM1(address, 0);
	roboclaw1.ForwardM2(address, 0);
	roboclaw2.ForwardM1(address, 0);
	roboclaw2.ForwardM2(address, 0);
}

 


  
 
// so the problem here is that the encmod value is being reassigned based on the current enc1 value which defeats the purpose
// need to be able to use encmod as the changing variable
// so while loop will begin, M1 turns on --> encmod is given a new value --> while loop checks itself

// now lets try doing it with the 4bil conversion

//y = enc1 + DEG

//if (y < 0)// went over numberline
 //{ y = 4294967295 + enc1 - DEG 
   //roboclaw.BackwardM1(address,60)
  //while (enc1 > 0 && enc1 < 4000000000) {}
  //while (enc1 > y){}
 // roboclaw.BackwardM1(address,0)
// }
 
//else if (y > 0) // went over numberline
  //{y = DEG - 4294967295 + enc1
  //roboclaw.ForwardM1(address,60)
  //while (enc1 < 4294967295 


// if (y < 0) //Went below zero
//{ y= 4294967295 + enc1 - DEG}
//else if  (y > 4294967295) // Went past zero
//{y= DEG - enc1}
//else {y= enc1 + DEG}


  
  



