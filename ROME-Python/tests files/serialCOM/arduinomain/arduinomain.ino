String inBound;
unsigned int x;

void setup()
{
  // code runs once
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop()
{
  // put your main code here, to run repeatedly:

  if(!Serial.available())
  {
    inBound = Serial.readStringUntil('\n');
    x = inBound.toInt();
    switch (x)
    {
      case 1:
        // statements
        Serial.write("Got 1");
        break;
      case 2:
        // statements
        Serial.write("Got 2");
        break;
      default:
        // statements
        Serial.write("Got something else");
        break;
    }
    Serial.flush();
  }
}
