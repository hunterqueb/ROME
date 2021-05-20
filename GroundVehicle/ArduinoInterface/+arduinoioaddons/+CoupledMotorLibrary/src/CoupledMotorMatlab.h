
#ifndef CoupledMotorLibrary_h
#define CoupledMotorLibrary_h

#include "LibraryBase.h"
#include "FinalMotorLibrary.h"
#include "AccelStepper.h"

//Debug statements
const char MSG_CREATE[]       PROGMEM = "CoupledMotorMatlab::mPointer[%d] = %d %d %d %f %f %f\n";
const char MSG_READ[]       PROGMEM = "CoupledMotorMatlab::mPointer[%d].read() = %d\n";
const char PID_BUGS[]        PROGMEM = "PID Output Voltage : %d\n";
const char MSG_SETPOINT[]       PROGMEM = "Setpoint recieved: %d\n";

#define MAX_MOTORS 4
#define MAX_STEPPERS 6

//Available commands through MATLAB interface
#define MOTOR_CREATE 0x03
#define GET_COUNTS 0x02
#define GET_RAD 0x00
#define GET_RADSEC 0x01
#define GET_COUNTSSEC 0x04
#define SET_RADSEC 0x05
#define UPDATE_MOTORS 0x06
#define UPDATE_STEPPERS 0x07

//GLOBAL VARIABLES
double Kp = 2.5,Ki = 60.25,Kd = 0.0;
double inputRadSec1, outputVoltage1, setpointRadSec1 = 0.0;
double inputRadSec2, outputVoltage2, setpointRadSec2 = 0.0;
double inputRadSec3, outputVoltage3, setpointRadSec3 = 0.0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

class CoupledMotorMatlab : public LibraryBase
{
    public:
    
    Motor* mPointer[MAX_MOTORS];
    AccelStepper* sPointer[MAX_STEPPERS];
    
    public:
        
    CoupledMotorMatlab(MWArduinoClass& a)
    {
        libName = "CoupledMotorLibrary/CoupledMotor";
        a.registerLibrary(this);
    }
    
    void setup()
    {
        AFMS.begin();
    }
    
    void loop()
    {
        //This loop will bring the motors to a desired setpoint by updating
        //the PID, as defined earlier in the code.
        if(mPointer[0] != NULL)
        {
            mPointer[0]->updateMotor();
        }
        
        if(mPointer[1] != NULL)
        {
            mPointer[1]->updateMotor();
        }
        
        if(mPointer[2] != NULL)
        {
            mPointer[2]->updateMotor();
        }
        
        if(sPointer[0] != NULL)
        {
            sPointer[0]->runSpeed();
        }
        
        if(sPointer[1] != NULL)
        {
            sPointer[1]->runSpeed();
        }
        
        if(sPointer[2] != NULL)
        {
            sPointer[2]->runSpeed();
        }
        
        if(sPointer[3] != NULL)
        {
            sPointer[3]->runSpeed();
        }
        
        if(sPointer[4] != NULL)
        {
            sPointer[4]->runSpeed();
        }
    }
    
    public:
    
    void commandHandler(byte cmdID,byte* dataIn,unsigned int payload_size)
    {
        switch (cmdID)
        {
            case MOTOR_CREATE:
            {
                byte ID, pinNumbers[2];
                ID = dataIn[0];

                for (byte i=0;i<2;i=i=i+1)
                {
                    pinNumbers[i] = dataIn[i+1];
                }
                
                if(ID == 0)
                {
                    mPointer[ID] = new Motor(1,pinNumbers[0],pinNumbers[1],
                                           3072,&inputRadSec1,&outputVoltage1,&setpointRadSec1,
                                           Kp,Ki,Kd,DIRECT);
                    mPointer[ID]->setAfms(&AFMS);
                    mPointer[ID]->registerMotor();
                    debugPrint(MSG_CREATE,ID,pinNumbers[0],pinNumbers[1],3702,Kp,Ki,Kd);
                }
                else if(ID == 1)
                {
                    mPointer[ID] = new Motor(2,pinNumbers[0],pinNumbers[1],
                                           3072,&inputRadSec2,&outputVoltage2,&setpointRadSec2,
                                           Kp,Ki,Kd,DIRECT);
                    mPointer[ID]->setAfms(&AFMS);
                    mPointer[ID]->registerMotor();
                    debugPrint(MSG_CREATE,ID,pinNumbers[0],pinNumbers[1],3702,Kp,Ki,Kd);
                }
                else if(ID == 2)
                {
                    mPointer[ID] = new Motor(3,pinNumbers[0],pinNumbers[1],
                                           3072,&inputRadSec3,&outputVoltage3,&setpointRadSec3,
                       s                    Kp,Ki,Kd,DIRECT);
                    mPointer[ID]->setAfms(&AFMS);
                    mPointer[ID]->registerMotor();
                    debugPrint(MSG_CREATE,ID,pinNumbers[0],pinNumbers[1],3702,Kp,Ki,Kd);
                }
                
                sendResponseMsg(cmdID,0,0);
                break;
            }
            case STEPPER_CREATE:
            {
                byte ID, pinNumbers[2];
                ID = dataIn[0];
                
                for (byte i=0;i<2;i=i=i+1)
                {
                    pinNumbers[i] = dataIn[i+1];
                }
                
                if(ID == 0)
                {
                    sPointer[ID] = new AccelStepper(pinNumbers[0],pinNumbers[1]);
                    sPointer[ID]->setMaxSpeed(1000);
                    sPointer[ID]->setSpeed(0);
                }
                else if(ID == 1)
                {
                    sPointer[ID] = new AccelStepper(pinNumbers[0],pinNumbers[1]);
                    sPointer[ID]->setMaxSpeed(1000);
                    sPointer[ID]->setSpeed(0);
                }
                else if(ID == 2)
                {
                    sPointer[ID] = new AccelStepper(pinNumbers[0],pinNumbers[1]);
                    sPointer[ID]->setMaxSpeed(1000);
                    sPointer[ID]->setSpeed(0);
                }
                else if(ID == 3)
                {
                    sPointer[ID] = new AccelStepper(pinNumbers[0],pinNumbers[1]);
                    sPointer[ID]->setMaxSpeed(1000);
                    sPointer[ID]->setSpeed(0);
                }
                else if(ID == 4)
                {
                    sPointer[ID] = new AccelStepper(pinNumbers[0],pinNumbers[1]);
                    sPointer[ID]->setMaxSpeed(1000);
                    sPointer[ID]->setSpeed(0);
                }
                
                sendResponseMsg(cmdID,0,0);
                break;
            }
            case GET_COUNTS:
            {
                byte ID = dataIn[0];
                
                if(mPointer[ID] == NULL)
                    break;
                
                int32_t count = mPointer[ID]->encoder->read();
                byte result[4];
                
                result[0] = (count & 0x000000ff);
                result[1] = (count & 0x0000ff00) >> 8;
                result[2] = (count & 0x00ff0000) >> 16;
                result[3] = (count & 0xff000000) >> 24;
                
                debugPrint(MSG_READ,ID,count);
                
                sendResponseMsg(cmdID,result,4);
                break;
            }
            case SET_RADSEC:
            {
                
                byte ID = dataIn[0];
                byte radSec[4];
                
                radSec[0] = dataIn[1];
                radSec[1] = dataIn[2];
                radSec[2] = dataIn[3];
                radSec[3] = dataIn[4];          
                
                if(ID == 0)
                {
                     setpointRadSec1 = *((float*)(radSec));
                }
                else if(ID == 1)
                {
                    setpointRadSec2 = *((float*)(radSec));
                }
                else if(ID == 2)
                {
                    setpointRadSec3 = *((float*)(radSec));
                }        

                sendResponseMsg(cmdID,0,0);
                break;
            }
            case UPDATE_MOTORS:
            {
                byte radSec1[4];
                byte radSec2[4];
                byte radSec3[4];
                
                radSec1[0] = dataIn[0];
                radSec1[1] = dataIn[1];
                radSec1[2] = dataIn[2];
                radSec1[3] = dataIn[3];
                
                radSec2[0] = dataIn[4];
                radSec2[1] = dataIn[5];
                radSec2[2] = dataIn[6];
                radSec2[3] = dataIn[7];
                
                radSec3[0] = dataIn[8];
                radSec3[1] = dataIn[9];
                radSec3[2] = dataIn[10];
                radSec3[3] = dataIn[11];
                
                setpointRadSec1 = *((float*)(radSec1));
                setpointRadSec2 = *((float*)(radSec2));
                setpointRadSec3 = *((float*)(radSec3));
                
                sendResponseMsg(cmdID,0,0);
                break;           
            }
            case UPDATE_STEPPERS:
            {
                byte countsSec1[4];
                byte countsSec2[4];
                byte countsSec3[4];
                byte countsSec4[4];
                byte countsSec5[4];
                
                byte countsSec1[0] = dataIn[0];
                byte countsSec1[1] = dataIn[1];
                byte countsSec1[2] = dataIn[2];
                byte countsSec1[3] = dataIn[3];
                
                byte countsSec2[0] = dataIn[4];
                byte countsSec2[1] = dataIn[5];
                byte countsSec2[2] = dataIn[6];
                byte countsSec2[3] = dataIn[7];
                
                byte countsSec3[0] = dataIn[8];
                byte countsSec3[1] = dataIn[9];
                byte countsSec3[2] = dataIn[10];
                byte countsSec3[3] = dataIn[11];
                
                byte countsSec4[0] = dataIn[12];
                byte countsSec4[1] = dataIn[13];
                byte countsSec4[2] = dataIn[14];
                byte countsSec4[3] = dataIn[15];
                
                byte countsSec5[0] = dataIn[16];
                byte countsSec5[1] = dataIn[17];
                byte countsSec5[2] = dataIn[18];
                byte countsSec5[3] = dataIn[19];
                
                Stepper1.setSpeed(*((float*)(countsSec1)));
                Stepper2.setSpeed(*((float*)(countsSec2)));
                Stepper3.setSpeed(*((float*)(countsSec3)));
                Stepper4.setSpeed(*((float*)(countsSec4)));
                Stepper5.setSpeed(*((float*)(countsSec5)));
                
                sendResponseMsg(cmdID,0,0);
                break;
            }
            /*
            case GET_COUNTSSEC:
            {
                byte ID;
                byte numEncoders = dataIn[0];
                byte result[4*numEncoders];
                int32_t oldCount[numEncoders];
                int32_t newCount[numEncoders];
                int32_t countDiff[numEncoders];
                
                for(size_t i = 0; i < numEncoders; i++)
                    {
                        ID = dataIn[i+1];
                        oldCount[i] = mPointer[ID].read();
                    }
                
                delay(getEncoderFreq());
                
                for(size_t i = 0; i < numEncoders; i++)
                    {
                        ID = dataIn[i+1];
                        newCount[i] = mPointer[ID].read();
                        countDiff[i] = newCount[i] - oldCount[i];
                    }
                
                for(size_t i =0; i < numEncoders; i = i+4)
                {
                    result[i] = (count & 0x000000ff);
                    result[i+1] = (count & 0x0000ff00) >> 8;
                    result[i+2] = (count & 0x00ff0000) >> 16;
                    result[i+3] = (count & 0xff000000) >> 24;
                {
                break;
            }
             */
            /*
            case GET_RAD:
            {
                byte ID = dataIn[0];
                double count = mPointer[ID]->getRad();
                
                byte result[4];
                
                result[0] = (count & 0x000000ff);
                result[1] = (count & 0x0000ff00) >> 8;
                result[2] = (count & 0x00ff0000) >> 16;
                result[3] = (count & 0xff000000) >> 24;
                
                sendResponseMsg(cmdID,result,4);
                break;
            }    
            case GET_RADSEC:
            {
                byte ID = dataIn[0];
                int32_t count = mPointer[ID]->getRadSec();
                
                byte result[4];
                
                result[0] = (count & 0x000000ff);
                result[1] = (count & 0x0000ff00) >> 8;
                result[2] = (count & 0x00ff0000) >> 16;
                result[3] = (count & 0xff000000) >> 24;
                
                sendResponseMsg(cmdID,result,4);
                break;
            }
             */
            default:
            {
                // Do nothing
                break;
            }
        }
    }
};
	
#endif