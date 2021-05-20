#include <FinalMotorLibrary.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

double input1, output1,input2, output2,input3, output3;

double Kp = 1.25,Ki = 31.25,Kd = 0.0;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Motor Motor1 = Motor(1,2,23,3072,&input1,&output1,&setpoint2,Kp,Ki,Kd,DIRECT);
Motor Motor2 = Motor(2,19,27,3072,&input2,&output2,&setpoint2,Kp,Ki,Kd,DIRECT);
Motor Motor3 = Motor(3,18,25,3072,&input3,&output3,&setpoint2,Kp,Ki,Kd,DIRECT);

double k2 = .0134; //N*m/A
double k3 = .014; //V/rpm
double Ra = 1.9; //ohms

double E1 = 12.0; //V
double E2 = 12.0; //V
double E3 = 12.0; //V

double b1[1024]; //
double b2[1024];
double b3[1024];

void setup() 
{
  Serial.begin(9600)

  AFMS.begin();

  Motor1.setAfms(&AFMS);
  Motor1.registerMotor();

  Motor2.setAfms(&AFMS);
  Motor2.registerMotor();

  Motor3.setAfms(&AFMS);
  Motor3.registerMotor();

  delay(2000);

  Motor1.setDuty(255);
  Motor2.setDuty(255);
  Motor3.setDuty(255);

  delay(2000);
}

void loop() 
{
  for (int i = 0;i < 1024;i++)
  {
    b1[i] = (k2/Ra)*E1/Motor1.getRadSec() - (k2*k3/Ra);
    b2[i] = (k2/Ra)*E2/Motor2.getRadSec() - (k2*k3/Ra);
    b3[i] = (k2/Ra)*E3/Motor3.getRadSec() - (k2*k3/Ra);
  }

}
