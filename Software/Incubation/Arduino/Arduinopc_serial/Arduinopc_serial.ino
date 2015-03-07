/* 1st byte is topic 
  
*/

int bytes[] = { 1, 2, 1 };
typedef void(*FunctionPointers)(char*);
FunctionPointers FunctionPointer[] = { &shutOff, &panAndTilt, &homing };
char data[2];

int RelayPin;
boolean turn_off;




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

void homing(char * data)
{
  
}

void setup() {
        
  //autoshutdown
    Serial.begin(9600);
    pinMode(RelayPin, OUTPUT);  
    
    //char data[2];
    //typedef void(*FunctionPointers)(char*);
    //FunctionPointer FunctionPointers[] = { &shutOff, &panAndTilt, &homing }

}
  
void loop() {
        
      if (Serial.available() > 0) {
          
        int topic = (int) Serial.read();
        
        for (int i = 0; i < bytes[topic]; i++) {
          data[i] = Serial.read();
        }
        
        handleSubscribers(topic, data);
      }
}


void handleSubscribers(int topic, char * data) {
  FunctionPointer[topic](data);
}
