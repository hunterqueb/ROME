#include<revisedMotorHeader.h>



rMotor::rMotor(uint8_t phaseA, uint8_t phaseB, double *input,double *output, double *setpoint,double Kp,double Ki,double
 Kd,int ControllerDirection) : pid(input,output,setpoint,Kp,Ki,Kd,ControllerDirection)
{
    encoder = new Encoder();
    numMotors = 0;

}


void rMotor::setDuty(uint8_t speed)
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




void rMotor::registerMotor()
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

        _pmotor = AFMS->getMotor(getNumMotors());

        addMotorCount();

    }
}

Adafruit_DCMotor *rMotor::getPmotor() const {
    return _pmotor;
}

void rMotor::setPmotor(Adafruit_DCMotor *pmotor) {
    _pmotor = pmotor;
}


void rMotor::setAfms(Adafruit_MotorShield *afms) {
    AFMS = afms;
}



int rMotor::getNumMotors() {
    return numMotors;
}

void rMotor::addMotorCount() {
    rMotor::numMotors++;
}





