/******************************************************************************** 
   Testing Zooming In and Out of Sony Camcorder 
   Wed Nov 5 2014
   How to build
   It was discovered that the zoom works by determining a potential 
   difference between the two pins attaced to the camcorder. So it doesn't matter
   which pins connect to which Gnd or Analog Out
   Also No PWN was used to control the camera zoom

BY: ADAM HEINS AND RAHUL RAWAT
********************************************************************************/ 

#define ZOOMIN 255 // gives maximum voltage to the pin (5V) 
// #define STILL 255/2  // we wanted a function to be able to hold the zoom in place. this didn't work so well.
#define ZOOMOUT 0 // gives 0V to pin

int focus = 2; //connect to Analog pin out 

void setup()
{
  pinMode (focus, OUTPUT) ; 
}

void loop()
{
  zoom(ZOOMIN,2000); 
  //zoom(STILL, 2000); 
  zoom(ZOOMOUT,2000);
}

void zoom(int val, int time) // zooms for a certain amount of time.
{
  analogWrite(focus, val);
  delay(time);
}

/* things to improve
  - create function that can zoom camera for a certain value
    - function of time & voltage most likely
    
