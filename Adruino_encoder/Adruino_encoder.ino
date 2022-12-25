int count = 0;
int interrupt0 = 0;
int phaseA = 2;
int phaseB = 3;

void setup()
{
  pinMode(phaseA, INPUT_PULLUP);
  pinMode(phaseB, INPUT_PULLUP);
  attachInterrupt(0, demxung, RISING);
  Serial.begin(9600);
}
void demxung()
{
  if(digitalRead(3) == LOW)
  {
    count++;  
  }  
  else
  {
    count--;  
  }
}
void loop()
{
  Serial.println(count);  
}
