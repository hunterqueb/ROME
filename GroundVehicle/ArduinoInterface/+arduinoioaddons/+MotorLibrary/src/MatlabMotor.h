
#ifndef MatlabMotorLibrary_h
#define MatlabMotorLibrary_h

#include "LibraryBase.h"
#include "NewMotorLibrary.h"

//Debug statements
const char MSG_CREATE[]       PROGMEM = "MotorMatlab::mPointer[%d] = %d %d %d %f %f %f\n";
const char MSG_READ[]       PROGMEM = "MotorMatlab::mPointer[%d].read() = %d\n";
const char PID_BUGS[]        PROGMEM = "PID Output Voltage : %d\n";
const char MSG_SETPOINT[]       PROGMEM = "Setpoint recieved: %d\n";

#define MAX_MOTORS 4

//Available commands through MATLAB interface
#define MOTOR_CREATE 0x03
#define GET_COUNTS 0x02
#define GET_RAD 0x00
#define GET_RADSEC 0x01
#define GET_COUNTSSEC 0x04
#define SET_RADSEC 0x05
#define UPDATE_MOTORS 0x06
#define UPDATE_VOLTAGES 0x07

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
double Kp = 2.5,Ki = 60.25,Kd = 0.0;

int mode = 0;
// Mode 1 is for PID control, mode 0 for voltage control.

class MatlabMotor : public LibraryBase
{
    public:
    
    NewMotor* mPointer[MAX_MOTORS];
    
    public:
        
    MatlabMotor(MWArduinoClass& a)
    {
        libName = "MotorLibrary/DCMotor";
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

            if (mode == 1)
            {
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
                

                mPointer[ID] = new NewMotor(ID+1,&AFMS,pinNumbers[0],pinNumbers[1],3072,2.6,60.0,0.0,DIRECT);
                mPointer[ID]->begin();
                debugPrint(MSG_CREATE,ID,pinNumbers[0],pinNumbers[1],3702,Kp,Ki,Kd);

                sendResponseMsg(cmdID,0,0);
                break;
            }
            case GET_COUNTS:
            {
                byte ID = dataIn[0];
                
                if(mPointer[ID] == NULL)
                    break;
                
                int32_t count = mPointer[ID]->getCounts();
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
                
                if (mPointer[ID]->getPIDMode() == 0)
                {
                    mPointer[ID]->setPIDMode(1);
                }
                
                byte radSec[4];
                
                radSec[0] = dataIn[1];
                radSec[1] = dataIn[2];
                radSec[2] = dataIn[3];
                radSec[3] = dataIn[4];          
                
                mPointer[ID]->setSetpoint(*((float*)(radSec)));

                sendResponseMsg(cmdID,0,0);
                break;
            }
//             case GET_RADSEC:
//                 byte ID = dataIn[0];
//                 int32_t count = mPointer[ID]->getRadSec();
//                 
//                 byte result[4];
//                 
//                 result[0] = (count & 0x000000ff);
//                 result[1] = (count & 0x0000ff00) >> 8;
//                 result[2] = (count & 0x00ff0000) >> 16;
//                 result[3] = (count & 0xff000000) >> 24;
//                 
//                 sendResponseMsg(cmdID,result,4);
//                 break;
                
            case UPDATE_MOTORS:
            {
                
                for (int i = 0; i < 3; i++)
                {
                    if (mPointer[i]->getPIDMode() == 0)
                    {
                        mPointer[i]->setPIDMode(1);
                    }
                }
                
                mode = 1;
                
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
                
                mPointer[0]->setSetpoint(*((float*)(radSec1)));
                mPointer[1]->setSetpoint(*((float*)(radSec2)));
                mPointer[2]->setSetpoint(*((float*)(radSec3)));
                
                sendResponseMsg(cmdID,0,0);
                break;           
            }
            case UPDATE_VOLTAGES:
            {
                /*
                mPointer[0]->setPIDMode(0);
                mPointer[1]->setPIDMode(0);
                mPointer[2]->setPIDMode(0);
                */
                
                byte volt1[2];
                byte volt2[2];
                byte volt3[2];
                                        
                for(int i=0;i<2;i++)
                {
                    volt1[i] = dataIn[i];
                }
                
                for(int i=2;i<4;i++)
                {
                    volt2[i-2] = dataIn[i];
                }
                
                for(int i=4;i<6;i++)
                {
                    volt3[i-4] = dataIn[i];
                }
                
                mPointer[0]->setDuty(*((int*)(volt1)));
                mPointer[1]->setDuty(*((int*)(volt2)));
                mPointer[2]->setDuty(*((int*)(volt3)));
                    
            }
            default:
            {
                // Do nothing
                break;
            }
        }
    }
};
	
#endif