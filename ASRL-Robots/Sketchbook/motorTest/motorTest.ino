#include <FinalMotorLibrary.h>

double setpoint = (3.0*3.14),setpoint2 = -1.0;

double input1, output1,input2, output2,input3, output3;

double Kp = 1.25,Ki = 31.25,Kd = 0.0;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Motor Motor1 = Motor(1,2,23,3072,&input1,&output1,&setpoint2,Kp,Ki,Kd,DIRECT);
Motor Motor2 = Motor(2,19,27,3072,&input2,&output2,&setpoint2,Kp,Ki,Kd,DIRECT);
Motor Motor3 = Motor(3,18,25,3072,&input3,&output3,&setpoint2,Kp,Ki,Kd,DIRECT);



void setup()
{
    Serial.begin(9600);
    
    AFMS.begin();

    Motor1.setAfms(&AFMS);
    Motor1.registerMotor();

    Motor2.setAfms(&AFMS);
    Motor2.registerMotor();

    Motor3.setAfms(&AFMS);
    Motor3.registerMotor();
    

    delay(2000);

    //Motor1.setDuty(0);
}

void loop()
{
    //long counts1 = Motor1.encoder->read();
    //long counts2 = Motor2.encoder->read();
    //long counts3 = Motor3.encoder->read();

    //Serial.print("Counts 1:");
    //Serial.println(counts1);
    //Serial.print("Counts 2:");
    //Serial.println(counts2);
    //Serial.print("Counts 3:");
    //Serial.println(counts3);
    
    Motor1.updateMotor(); 
    //Motor1.setDuty(50);
   // Motor1.printPIDInfo();
    Motor2.updateMotor();
    //Motor2.setDuty(50);
    //Motor2.printPIDInfo();
    Motor3.updateMotor(); 
    //Motor3.setDuty(50);
    //Motor3.printPIDInfo();
    
}
