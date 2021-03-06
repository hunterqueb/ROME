
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

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
double Kp = 2.5,Ki = 60.25,Kd = 0.0;

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
        if(mPointer[0] != NULL)
        {
            mPointer[0]->updateMotor();
            //mPointer[0]->setDuty(50);
        }
        
        if(mPointer[1] != NULL)
        {
            mPointer[1]->updateMotor();
            //mPointer[1]->setDuty(50);
        }
        
        if(mPointer[2] != NULL)
        {
            mPointer[2]->updateMotor();
            //mPointer[2]->setDuty(50);
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
                byte radSec[4];
                
                radSec[0] = dataIn[1];
                radSec[1] = dataIn[2];
                radSec[2] = dataIn[3];
                radSec[3] = dataIn[4];          
                
                mPointer[ID]->setSetpoint(*((float*)(radSec)));

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
                
                mPointer[0]->setSetpoint(*((float*)(radSec1)));
                mPointer[1]->setSetpoint(*((float*)(radSec2)));
                mPointer[2]->setSetpoint(*((float*)(radSec3)));
                
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