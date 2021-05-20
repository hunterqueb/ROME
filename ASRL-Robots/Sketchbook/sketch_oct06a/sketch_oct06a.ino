
#include <AccelStepper.h>

AccelStepper stepper1(6,5);
//AccelStepper stepper2(4,5);
//AccelStepper stepper3(8,9);
//AccelStepper stepper4(6,7);
//AccelStepper stepper5(7,8);

void setup()
{  
   stepper1.setMaxSpeed(1000);
   stepper1.setSpeed(100);  

   //stepper2.setMaxSpeed(1000);
   //stepper2.setSpeed(-100);  

   /*
   stepper3.setMaxSpeed(1000);
   stepper3.setSpeed(50);  

   stepper4.setMaxSpeed(1000);
   stepper4.setSpeed(50);  

   stepper5.setMaxSpeed(1000);
   stepper5.setSpeed(50);
   */
}

void loop()
{ 
   //stepper2.runSpeed();
   stepper1.runSpeed();
   /*
   stepper3.runSpeed();
   stepper4.runSpeed();
   stepper5.runSpeed();
   */
}
