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

void calcThro()
{
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

#define address 0x80

void setup() {
  
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  attachInterrupt(0, calcThro, CHANGE);
  attachInterrupt(1, calcAile, CHANGE);
  attachInterrupt(2, calcElev, CHANGE);
  attachInterrupt(3, calcRudd, CHANGE);
  attachInterrupt(5, calcGear, CHANGE);
  
  Serial.begin(9600);
  
  digitalWrite(13, HIGH);

  Serial.println("Start!");
}

void loop() {
  if(pulseTimeThro <= 1033) // Direction
    pulseTimeThro = 1033;
  else if(pulseTimeThro >= 1992)
    pulseTimeThro = 1992;
    
  if(pulseTimeRudd <= 1009)
    pulseTimeRudd = 1009;
  else if(pulseTimeRudd >= 1607)
    pulseTimeRudd = 1607;
  
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;
  
  Serial.print(pulseTimeThro);
  Serial.print("\t");
  Serial.print(pulseTimeAile);
  Serial.print("\t");
  Serial.print(pulseTimeElev);
  Serial.print("\t");
  Serial.print(pulseTimeRudd);
  Serial.print("\t");
  Serial.println(pulseTimeGear);
}
