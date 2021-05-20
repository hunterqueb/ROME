#include "FinalMotorLibrary.h"

double input,output,setpoint = 3.0,input2,output2,setpoint2 = 3.0;
double Kp = 2.5,Ki = 62.5,Kd = 0.0;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Motor Motor1 = Motor(2,23,3072,&input,&output,&setpoint,Kp,Ki,Kd,DIRECT);
//Motor Motor2(18,25,3072,&input2,&output2,&setpoint2,Kp,Ki,Kd,DIRECT);

void setup() 
{
  AFMS.begin();
  Motor1.setMotor(AFMS.getMotor(1));

  Serial.begin(9600);
  Serial.println("A lot of words");
  Motor1.setDuty(0);
  //Motor2.setDuty(0);

  delay(2000);
  Motor1.setDuty(210);
  //delay(2000);
  Motor1.setDuty(0);
  //Motor2.setDuty(210);
}

void loop() 
{
  //Motor2.setDuty(210);// put your main code here, to run repeatedly:
}
