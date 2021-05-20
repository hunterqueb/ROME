#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

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

  delay(1000);
}

double zArray[500];
//double z2Array[1000];
double tArray[500];
int index = 0;

void loop() {
  // put your main code here, to run repeatedly:
  
  sensors_event_t linearAccelData, accelerometerData, gravityData;
  
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  tArray[index] = millis();

  
//
//  double x = linearAccelData.acceleration.x;
//  double y = linearAccelData.acceleration.y;
    zArray[index] = linearAccelData.acceleration.z;

//  double x2 = accelerometerData.acceleration.x;
//  double y2 = accelerometerData.acceleration.y;
//    z2Array[index] = accelerometerData.acceleration.z;

//  Serial.print(x);
//  Serial.print(",");
//  Serial.print(y);
//  Serial.print(",");
//  Serial.print(z);
//  Serial.print(",");

//  Serial.print(x2);
//  Serial.print(",");
//  Serial.print(y2);
//  Serial.print(",");
//  Serial.print(z2);
//  Serial.print(",");
//  Serial.println();
  
  //delay(BNO055_SAMPLERATE_DELAY_MS);
  index++;

  if(index == 499)
  {
    for(int i = 0;i< 500;i++)
    {
      Serial.print(zArray[i]);
      Serial.print(",");
//      Serial.print(z2Array[i]);
//      Serial.print(",");
      Serial.print(tArray[i]);
      Serial.println();
    }
  }
}
