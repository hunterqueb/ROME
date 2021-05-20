#include "Arduino.h"
#include "Encoder.h"
#include "PID_v1.h"
#include "Adafruit_MotorShield.h"

class NewMotor
{
public: 
    
    NewMotor(int id, Adafruit_MotorShield *_pShield,uint8_t phaseA, uint8_t phaseB, unsigned int encoderCPR,double Kp,double Ki,double Kd,int dir);
    void begin();
    
    void setDuty(int speed);
    void setSetpoint(float setpoint);
    void updateMotor();
    void setEncoderFreq(int);
    int getEncoderFreq();
	void setPIDMode(int);
	int getPIDMode();
    
    float getDeg();
    float getDegSec();

    float getRad();
    float getRadSec();

    float getRevs();
    float getRevsSec();
    
    long getCounts();
    float getCountsSec();

private:

    // Motor stuff
    int _id;
    Adafruit_MotorShield *_pShield;
    Adafruit_DCMotor *_pMotor;
    
    // Encoder stuff
    unsigned int _encoderCPR;
    Encoder _encoder;
    int _freq;
    long _counts;
    long _lastCount;
    
    // Keeps track of last calculation time for counts/second.
    unsigned long _lastTime = 0;
    float _countsSec;
    
    float _deg;
    float _degSec;

    float _rad;
    float _radSec;

    float _revs;
    float _revsSec;
    
    // PID stuff
    double _input = 0.0, _output = 0.0, _setpoint = 0.0;
    int _dir;
    PID _pid;
};





