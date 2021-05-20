
#include <NewMotorLibrary.h>

/* This example test should cause all three motor to spin at 2 radians/second CCW*/

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
NewMotor Motor1 = NewMotor(1,&AFMS,2,23,3072,2.6,60.0,0,DIRECT);
NewMotor Motor2 = NewMotor(2,&AFMS,19,27,3072,2.6,60.0,0,DIRECT);
NewMotor Motor3 = NewMotor(3,&AFMS,18,25,3072,2.6,60.0,0,DIRECT);

void setup() 
{
  Serial.begin(9600);
  Serial.println("New Motor Library Test!");

  Motor1.begin();
  Motor2.begin();
  Motor3.begin();

  Motor1.setSetpoint(2.0);
  Motor2.setSetpoint(2.0);
  Motor3.setSetpoint(2.0);
}

void loop() 
{
  Motor1.updateMotor();
  Motor2.updateMotor();
  Motor3.updateMotor();
}
