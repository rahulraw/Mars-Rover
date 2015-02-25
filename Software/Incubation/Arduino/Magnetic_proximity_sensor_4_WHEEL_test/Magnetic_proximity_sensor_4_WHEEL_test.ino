int SensorInput = 7;//Pin used for sensor

void setup()
{
  Serial.begin(9600);
  pinMode(SensorInput, INPUT);
  pinMode(13, OUTPUT);
}

void loop()
{
 
  digitalWrite(13,digitalRead(SensorInput));
  Serial.println(digitalRead(SensorInput));
}


