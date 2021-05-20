#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 0;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup(void)
{
  Serial.begin(2000000);
  Serial.println("Orientation Sensor Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  Serial.println("t,Ax,Ay,Az,t,Ex,Ey,Ez,t,Wx,Wy,Wz");

  delay(1000);
}

void loop(void)
{

  while(millis() <= 60000) {
  sensors_event_t orientationData , angVelocityData , linearAccelData;
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

  printEvent(&linearAccelData);
  printEvent(&orientationData);
  printEvent(&angVelocityData);
  Serial.println();

  //delay(BNO055_SAMPLERATE_DELAY_MS);

  }
}

void printEvent(sensors_event_t* event)
{
  double x = -1000000, y = -1000000 , z = -1000000, t = 1000000; //dumb values, easy to spot problem
  x = event->orientation.x;
  y = event->orientation.y;
  z = event->orientation.z;
  t = event->timestamp;

  Serial.print(t);
  Serial.print(",");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.print(z);
  Serial.print(",");
}
