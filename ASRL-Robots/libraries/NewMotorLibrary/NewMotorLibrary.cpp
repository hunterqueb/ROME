#include"NewMotorLibrary.h"

// Constructor
NewMotor::NewMotor(int id, Adafruit_MotorShield *pShield, uint8_t phaseA, uint8_t phaseB, unsigned int encoderCPR,double Kp,double Ki,double Kd,int dir):_encoder(phaseA,phaseB),_pid(&_input,&_output,&_setpoint,Kp,Ki,Kd,dir)
{
    
    // Motor stuff
    _id = id;
    _pShield = pShield;
    
    // Encoder Stuff
    _encoderCPR = encoderCPR;
}

// Initialization function for Motor
void NewMotor::begin()
{
    _pShield->begin();
    _pMotor = _pShield->getMotor(_id);
    setEncoderFreq(20);

    _pid.SetOutputLimits(-255,255);
    _pid.SetMode(1);
}

// Sets a setpoint value for the velocity PID
void NewMotor::setSetpoint(float setpoint)
{
    _setpoint = setpoint;
}

// Updates velocity PID using encoder measurment. Should run in looping function.
void NewMotor::updateMotor()
{
    _input = getRadSec();
    Serial.println(_input);
    _pid.Compute();
    setDuty(_output);
}

// Sets motor duty cycle, 0 = 0%, 255 = 100%
void NewMotor::setDuty(int speed)
{
    if(speed < 0)
    {
        speed = speed*-1;
        _pMotor->setSpeed(speed);
        _pMotor->run(BACKWARD);
    }
    else
    {
        _pMotor->setSpeed(speed);
        _pMotor->run(FORWARD);
    }
}

// Sets sample time in ms for velocity and acceleration calculations
void NewMotor::setEncoderFreq(int freq)    
{
    _freq = freq;
}

// Gets sample time in ms for velocity and acceleration calculations
int NewMotor::getEncoderFreq()
{
    return _freq;
}

// Calculates and returns counts on encoder
long NewMotor::getCounts()
{
    return _encoder.read();
}

// Calculates and returns counts per second on encoder over given sample time
float NewMotor::getCountsSec()
{

    unsigned long now = millis();
    if((now - _lastTime) >= _freq)
    {
        _counts = _encoder.read();
        _countsSec = (_counts - _lastCount) * (1000.0/_freq);

        _lastTime = now;
        _lastCount = _counts;
    }
    return _countsSec;
}

// Returns degrees of motor based on counts
float NewMotor::getDeg()
{
    _deg = (_encoder.read()%3072)*(360.0/3072.0);
    return _deg;
}

// Returns radians of motor based on counts
float NewMotor::getRad()
{
    _rad = getRevs()*2.0*3.14159;
    return _rad;
}

// Returns radians/second of motor based on counts
float NewMotor::getRadSec()
{
    _radSec = getRevsSec()*2.0*3.14159;
    return _radSec;
}

float NewMotor::getRevs()
{
    _revs = _encoder.read()/_encoderCPR;
    return _revs;
}

float NewMotor::getRevsSec()
{
    _revsSec = getCountsSec()/_encoderCPR;
    return _revsSec;
}