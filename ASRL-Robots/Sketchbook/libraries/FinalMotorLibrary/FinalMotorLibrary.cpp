#include"FinalMotorLibrary.h"



NewMotor::Motor(int id, uint8_t phaseA, uint8_t phaseB, unsigned int encoderCPR,double Kp,double Ki,double
 Kd,int ControllerDirection)
{
    encoder = new Encoder(phaseA, phaseB);
    pid = new PID(_input,_output,_setpoint,Kp,Ki,Kd,ControllerDirection);
    motorID = id;
    numMotors = 0;
	pid->SetOutputLimits(-255,255);


    pid->SetMode(1);

    //Encoder Setup
    _phaseA = phaseA;
    _phaseB = phaseB;
    _encoderCPR = encoderCPR;
    setEncoderFreq(20);//time between updates for encoder

}


void NewMotor::updateMotor()
{

    setpidIn(getRadSec());
    pid->Compute();
    setDuty(getpidOut());

}

void NewMotor::setDuty(int speed)
{
    if(speed < 0)
    {
        speed = speed*-1;
        _pmotor->setSpeed(speed);
        _pmotor->run(BACKWARD);
        //Serial.println(speed);
    }
    else
    {
        _pmotor->setSpeed(speed);
        _pmotor->run(FORWARD);
        //Serial.println(speed);
    }

}

void NewMotor::setEncoderFreq(int freq)    //Sets sample time in ms for velocity and acceleration calculations//
{
    _freq = freq;
}

int NewMotor::getEncoderFreq()
{
    return _freq;
}



float NewMotor::getCountsSec() //Calculates counts per second over given sample time//
{

    unsigned long now = millis();
    if((now - _lastTime) >= _freq)
    {
        _counts = encoder->read();
        _countsSec = (_counts - _lastCount) * (1000.0/_freq);

        _lastTime = now;
        _lastCount = _counts;
    }
    return _countsSec;

    /*
    //OTHER
    long oldCount = read();
    delay(_freq);
    long newCount = read();
    long countDiff = newCount-oldCount;
    _countsSec = countDiff/_freq;
    */
}


float NewMotor::getDeg()   //Returns degrees 0-360 of motor based on counts//
{
    _deg = (encoder->read()%3072)*(360.0/3072.0);
    return _deg;
}

float NewMotor::getRad()
{
    _rad = getRevs()*2.0*3.14159;
    return _rad;
}

float NewMotor::getRadSec()
{
    _radSec = getRevsSec()*2.0*3.14159;
    return _radSec;
}

float NewMotor::getRevs()
{
    _revs = encoder->read()/_encoderCPR;
    return _revs;
}

float NewMotor::getRevsSec()
{
    _revsSec = getCountsSec()/_encoderCPR;
    return _revsSec;
}


Adafruit_DCMotor *NewMotor::getPmotor() const {
    return _pmotor;
}

void NewMotor::setPmotor(Adafruit_DCMotor *pmotor) {
    _pmotor = pmotor;
}







