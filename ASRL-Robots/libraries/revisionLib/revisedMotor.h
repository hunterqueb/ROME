#include "Arduino.h"
#include "Encoder.h"
#include "PID_v1.h"
#include "Adafruit_MotorShield.h"



class rMotor
{

public:

    rMotor(int id, uint8_t phaseA, uint8_t phaseB, unsigned int encoderCPR, double *input,double *output, double *setpoint,double Kp,double Ki,double Kd,int ControllerDirection);


    Encoder *encoder;
    PID *pid;



    //Getters and setters
    Adafruit_DCMotor *getPmotor() const;

    void setPmotor(Adafruit_DCMotor *pmotor);

    void setAfms(Adafruit_MotorShield *afms);

    int getNumMotors();

    void updateMotor();

    void registerMotor();

    void setDuty(int);

    float getCountsSec();

    double getpidOut();

    void setEncoderFreq(int freq);

    int getEncoderFreq();

    double getSetPoint();

    void setpidIn(double in);

    void printPIDInfo();




    float getDeg();
    float getDegSec();

    float getRad();
    float getRadSec();

    float getRevs();
    float getRevsSec();


private:

    Adafruit_DCMotor *_pmotor;

    Adafruit_MotorShield *AFMS;

    void addMotorCount();

    int numMotors;

    int motorID;

    byte _phaseA;
    byte _phaseB;

    double* pidOut;
    double* pidIn;
    double* setPointIn;

    //motor attributes
    unsigned int _encoderCPR;

    //encoder measurements
    int _freq;
    long _counts;
    long _lastCount;
    unsigned long _lastTime = 0; //Keeps track of last calculation time for counts/second.

    float _countsSec;
    float _lastCountsSec;
    unsigned long _lastTime1; //Keeps track of last calculation time for counts/second^2


    float _deg;
    float _degSec;

    float _rad;
    float _radSec;

    float _revs;
    float _revsSec;


};





