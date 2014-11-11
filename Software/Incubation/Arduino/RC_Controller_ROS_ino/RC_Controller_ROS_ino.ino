#define CHANNEL_1_PIN 2 //Throttle
#define CHANNEL_2_PIN 3 //Aile
#define CHANNEL_3_PIN 21 //Elev
#define CHANNEL_4_PIN 20 //Rudd
#define CHANNEL_5_PIN 18 //Gear Switch

#define SWITCH_THRESHOLD    1600
#define SWITCH_THRESHOLD_AILE    1200

#include <ros.h>
#include <std_msgs/Float32.h> 
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle  nh;

//Set up the ros node and publisher
std_msgs::Float32MultiArray RC_msg;
ros::Publisher pub_RC("RCValues", &RC_msg);
char dim0_label[] = "RCValues";

volatile unsigned long timerStartThro;
volatile unsigned long timerStartAile;
volatile unsigned long timerStartElev;
volatile unsigned long timerStartRudd;
volatile unsigned long timerStartGear;

volatile int pulseTimeThro = 1512;
volatile int pulseTimeAile;
volatile int pulseTimeElev = 1512;
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
      if (check_Thro > 900 && check_Thro < 2150) //&& abs(check_Thro - pulseTimeThro) < max_stick_change)
      {
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
      if (check_Aile > 900 && check_Aile < 2150) //&& abs(check_Aile - pulseTimeAile) < max_stick_change)
      {
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
      if (check_Elev > 900 && check_Elev < 2150) //&& abs(check_Elev - pulseTimeElev) < max_stick_change)
      {
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
      if (check_Rudd > 900 && check_Rudd < 2150) //&& abs(check_Rudd - pulseTimeRudd) < max_stick_change)
      {
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

void setup() {

  nh.initNode();
  RC_msg.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  RC_msg.layout.dim[0].label = dim0_label;
  RC_msg.layout.dim[0].size = 4;
  RC_msg.layout.dim[0].stride = 1*4;
  RC_msg.layout.data_offset = 0;
  RC_msg.data_length = 4;
  RC_msg.data = (float *)malloc(sizeof(float)*4);
  nh.advertise(pub_RC);
  
  
  //timer_start = 0;
  attachInterrupt(0, calcThro, CHANGE);
  attachInterrupt(1, calcAile, CHANGE);
  attachInterrupt(2, calcElev, CHANGE);
  attachInterrupt(3, calcRudd, CHANGE);
  attachInterrupt(5, calcGear, CHANGE);

  Serial.begin(9600);

  Serial.println("Waiting.......");
  //do the switch flick thing to preceed with controlling the robot
  //while (pulseTimeGear > SWITCH_THRESHOLD);
  //while (pulseTimeGear < SWITCH_THRESHOLD);

  Serial.println("Start!");
}

void loop() {

// reading in from the remote control
  if(pulseTimeThro <= 1033) // Direction
    pulseTimeThro = 1033;
  else if(pulseTimeThro >= 1992)
    pulseTimeThro = 1992;

  direct = map(pulseTimeThro, 1033, 1992, -90, 90); // rc direction stick position in degrees
  if (direct < 3 || direct > -3)
    direct = 0;
  
  if (spd < 3 || spd > -3)
    spd = 0;
  
  nh.spinOnce();
  
  if(pulseTimeElev <= 1120)
    pulseTimeElev = 1120;
  else if(pulseTimeElev >= 2000)
    pulseTimeElev = 2000;

  spd = map(pulseTimeElev, 1120, 2000, -127, 127); // convert rc stick speed to speed values (-127 to 127)

  RC_msg.data[0] = spd;
  RC_msg.data[1] = direct;
  RC_msg.data[2] = 0;
  RC_msg.data[3] = 0;
    
  Serial.println(spd);
  pub_RC.publish(&RC_msg);
  nh.spinOnce();
  delay(50);
  
}
