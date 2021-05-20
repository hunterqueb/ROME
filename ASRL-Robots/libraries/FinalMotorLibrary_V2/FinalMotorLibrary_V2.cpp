#include"FinalMotorLibrary.h"



Motor::Motor(int id, uint8_t phaseA, uint8_t phaseB, unsigned int encoderCPR, double *input,double *output, double *setpoint,double Kp,double Ki,double
 Kd,int ControllerDirection)
{
    encoder = new Encoder(phaseA, phaseB);
    pid = new PID(input,output,setpoint,Kp,Ki,Kd,ControllerDirection);
    motorID = id;
    numMotors = 0;
    pidOut = output;
    pidIn = input;
    setPointIn = setpoint;
	pid->SetOutputLimits(-255,255);



    pid->SetMode(1);

    //Encoder Setup
    _phaseA = phaseA;
    _phaseB = phaseB;
    _encoderCPR = encoderCPR;
    setEncoderFreq(20);//time between updates for encoder

}

void Motor::printPIDInfo()
{
    Serial.println("PID Data:");
    Serial.print("Motor ID = ");
    Serial.println(motorID);
    Serial.print("Input = ");
    Serial.println(*pidIn);
    Serial.print("Output = ");
    Serial.println(*pidOut);
    Serial.print("Set Point = ");
    Serial.println(*setPointIn);
    Serial.print("Rad per second = ");
    Serial.println(getRadSec());
    Serial.println("");
    Serial.println("");

}


void Motor::updateMotor()
{

    setpidIn(getRadSec());
    pid->Compute();
    setDuty(getpidOut());

}

void Motor::setDuty(int speed)
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




void Motor::registerMotor()
{
    if(!AFMS)
    {
        //If we get Here the motor shield has not been set
    }

    if(!_pmotor)
    {
        //if we get here than we need to create a new motor with the shield


        if(getNumMotors() == 4)
        {
            //If we get here than we cant register anymore motors since we are already at 4
            return;
        }

        _pmotor = AFMS->getMotor(motorID);

        addMotorCount();

    }
}

void Motor::setEncoderFreq(int freq)    //Sets sample time in ms for velocity and acceleration calculations//
{
    _freq = freq;
}

int Motor::getEncoderFreq()
{
    return _freq;
}



float Motor::getCountsSec() //Calculates counts per second over given sample time//
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


float Motor::getDeg()   //Returns degrees 0-360 of motor based on counts//
{
    _deg = (encoder->read()%3072)*(360.0/3072.0);
    return _deg;
}

float Motor::getRad()
{
    _rad = getRevs()*2.0*3.14159;
    return _rad;
}

float Motor::getRadSec()
{
    _radSec = getRevsSec()*2.0*3.14159;
    return _radSec;
}

float Motor::getRevs()
{
    _revs = encoder->read()/_encoderCPR;
    return _revs;
}

float Motor::getRevsSec()
{
    _revsSec = getCountsSec()/_encoderCPR;
    return _revsSec;
}


Adafruit_DCMotor *Motor::getPmotor() const {
    return _pmotor;
}

void Motor::setPmotor(Adafruit_DCMotor *pmotor) {
    _pmotor = pmotor;
}


void Motor::setAfms(Adafruit_MotorShield *afms) {
    AFMS = afms;
}



int Motor::getNumMotors() {
    return numMotors;
}

void Motor::addMotorCount() {
    numMotors++;
}

double Motor::getpidOut()
{
    return *pidOut;
}

void Motor::setpidIn(double in)
{
    *pidIn = in;
}

double Motor::getSetPoint()
{
    return *setPointIn;
}





