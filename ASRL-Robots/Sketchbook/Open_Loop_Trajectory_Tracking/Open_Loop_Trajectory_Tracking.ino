#include <FinalMotorLibrary.h>

double inputRadSec1, outputVoltage1, setpointRadSec1;
double inputRadSec2, outputVoltage2, setpointRadSec2;
double inputRadSec3, outputVoltage3, setpointRadSec3;

double Kp = 2.5,Ki = 60,Kd = 0.0;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Motor Motor1 = Motor(1,2,23,3072,&inputRadSec1,&outputVoltage1,&setpointRadSec1,Kp,Ki,Kd,DIRECT);
Motor Motor2 = Motor(2,19,27,3072,&inputRadSec2,&outputVoltage2,&setpointRadSec2,Kp,Ki,Kd,DIRECT);
Motor Motor3 = Motor(3,18,25,3072,&inputRadSec3,&outputVoltage3,&setpointRadSec3,Kp,Ki,Kd,DIRECT);

float theta = 0, pi = 3.14159, L = .115, R = .05;

float pTheta1 = -sin(theta);
float pTheta2 = cos(theta);
float pTheta3 = L;
float pTheta4 = -sin((pi/3)-theta);
float pTheta5 = -cos((pi/3)-theta);
float pTheta6 = L;
float pTheta7 = sin((pi/3)+theta);
float pTheta8 = -cos((pi/3)+theta);
float pTheta9 = L;

void setup() 
{
  Serial.begin(9600);
  Serial.println("EVERYTHING IS JUST STARTING");

  AFMS.begin();

  
  Motor1.setAfms(&AFMS);
  Motor1.registerMotor();

  Motor2.setAfms(&AFMS);
  Motor2.registerMotor();

  Motor3.setAfms(&AFMS);
  Motor3.registerMotor();

  delay(2000);
}

float lastTime;
float metSec1,metSec2,metSec3;

void loop() 
{
  float t = (millis())/1000.0;
  
  if((t-lastTime) > .02)
  {
    float xPath = cos(.25*t), yPath = sin(.25*t), thetaPath = 0;  //Period = 8 pi 
    float xPathPrime = -.25*sin(.25*t), yPathPrime = .25*cos(.25*t), thetaPathPrime = 0;


    metSec1 = pTheta1*(xPathPrime) + pTheta2*(yPathPrime) + pTheta3*(thetaPathPrime); 
    metSec2 = pTheta4*(xPathPrime) + pTheta5*(yPathPrime) + pTheta6*(thetaPathPrime);
    metSec3 = pTheta7*(xPathPrime) + pTheta8*(yPathPrime) + pTheta9*(thetaPathPrime);

    lastTime = t;
  }

  setpointRadSec1 = metSec1/R;
  setpointRadSec2 = metSec2/R;
  setpointRadSec3 = metSec3/R;

  Motor1.updateMotor(); 
  Motor2.updateMotor();
  Motor3.updateMotor();
}
