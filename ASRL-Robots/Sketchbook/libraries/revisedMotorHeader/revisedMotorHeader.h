#include "Arduino.h"
#include "Encoder.h"
#include "PID_v1.h"
#include "Adafruit_MotorShield.h"



class rMotor
{

public:

    rMotor(uint8_t phaseA, uint8_t phaseB, double *input,double *output, double *setpoint,double Kp,double Ki,double Kd,int ControllerDirection);


    Encoder encoder;
    PID pid;


    //Getters and setters
    Adafruit_DCMotor *getPmotor() const;

    void setPmotor(Adafruit_DCMotor *pmotor);

    void setAfms(Adafruit_MotorShield *afms);


    static int getNumMotors();

    void registerMotor();

    void setDuty(uint8_t);




private:


    Adafruit_DCMotor *_pmotor;

    static Adafruit_MotorShield *AFMS;

    static void addMotorCount();

    static int numMotors;


};





