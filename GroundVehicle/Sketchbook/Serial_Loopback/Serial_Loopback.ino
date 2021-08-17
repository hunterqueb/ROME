void setup() 
{
  Serial.begin(19200);

}

void loop() 
{
  if (Serial.available() > 0)
  {
    char c = Serial.read();
    Serial.write(c);
  }
}
