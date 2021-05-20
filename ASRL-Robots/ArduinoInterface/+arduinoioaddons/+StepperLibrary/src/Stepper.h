
#ifndef Stepper_h
#define Stepper_h

#include "LibraryBase.h"
#include "AccelStepper.h"

//Debug Messages
const char MSG_CREATE[]       PROGMEM = "Stepper created at pins %d,%d.\n";
const char MSG_SPEED[]        PROGMEM = "New stepper speed set for stepper %d.";
const char MSG_STATES[]       PROGMEM = "New position (%ld) and velocity set for stepper %d.";

#define MAX_STEPPERS 6

//Available commands through MATLAB interface
#define STEPPER_CREATE 0x01
#define SET_RADSEC 0x02
#define UPDATE_STEPPERS 0x03
#define SET_STATES 0x04
#define UPDATE_STATES 0x05
#define CALIBRATE 0x06
#define READ 0x07
#define SET 0x08
#define READ_ALL 0x09

class Stepper : public LibraryBase
{
    public:

    AccelStepper* sPointer[MAX_STEPPERS];

    // posMode = 1 refers to position and velocity based control.
    // posMode = 0 refers to velocity based control only.
    int posMode = 0;

    // Expressed in Joint frame
    long steps[6] = {0,0,0,0,0,0};
    float stepsSec[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

    // Steps per degree for each motor
    float stepsDeg[6] = {1/.022368421,1/.018082192,
                         1/.017834395,1/.021710526,
                         1/.045901639,1/.046792453};

    // Limits of each joint, degrees from zero. Expressed in Motor frame.
    // J2,J3,and J5 reversed here.
    float limits[6] = {170.0,132.0,-141.0,-155.0,105.0,-155.0};

    public:

    Stepper(MWArduinoClass& a)
    {
        libName = "StepperLibrary/Stepper";
        a.registerLibrary(this);
    }

    void setup()
    {
    }

    void loop()
    {
        if(posMode = 1)
        {
            if(sPointer[0] != NULL)
            {
                sPointer[0]->moveTo(steps[0]);
                sPointer[0]->setSpeed(stepsSec[0]);
                sPointer[0]->runSpeedToPosition();
            }
            if(sPointer[1] != NULL)
            {
                // J2 is reversed
                // Make transformation to Motor frame
                sPointer[1]->moveTo(-steps[1]);
                sPointer[1]->setSpeed(stepsSec[1]);
                sPointer[1]->runSpeedToPosition();
            }
            if(sPointer[2] != NULL)
            {   // J3 is reversed
                // Make transformation to Motor frame
                sPointer[2]->moveTo(-steps[2]);
                sPointer[2]->setSpeed(-stepsSec[2]);
                sPointer[2]->runSpeedToPosition();
            }
            if(sPointer[3] != NULL)
            {
                sPointer[3]->moveTo(steps[3]);
                sPointer[3]->setSpeed(stepsSec[3]);
                sPointer[3]->runSpeedToPosition();
            }
            if(sPointer[4] != NULL)
            {   // J5 is reversed
                // Make transformation to Motor frame
                sPointer[4]->moveTo(-steps[4]);
                sPointer[4]->setSpeed(-stepsSec[4]);
                sPointer[4]->runSpeedToPosition();
            }
            if(sPointer[5] != NULL)
            {
                sPointer[5]->moveTo(steps[5]);
                sPointer[5]->setSpeed(stepsSec[5]);
                sPointer[5]->runSpeedToPosition();
            }
        }
        else
        {
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

            if(sPointer[5] != NULL)
            {
                sPointer[5]->runSpeed();
            }
        }
    }

    public:

    void commandHandler(byte cmdID,byte* dataIn,unsigned int payload_size)
    {
        switch (cmdID)
        {
            case STEPPER_CREATE:
            {
                byte ID, pinNumbers[2];
                ID = dataIn[0];

                for (byte i=0;i<2;i=i=i+1)
                {
                    pinNumbers[i] = dataIn[i+1];
                }

                sPointer[ID] = new AccelStepper(AccelStepper::DRIVER,pinNumbers[0],pinNumbers[1]);
                sPointer[ID]->setMaxSpeed(1000);
                sPointer[ID]->setSpeed(0);

                debugPrint(MSG_CREATE,pinNumbers[0],pinNumbers[1]);
                sendResponseMsg(cmdID,0,0);
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


                sPointer[ID]->setSpeed(*((float*)(radSec)));

                debugPrint(MSG_SPEED,ID);
                sendResponseMsg(cmdID,0,0);
                break;
            }
            case SET_STATES:
            {
                 byte ID = dataIn[0];
                 posMode = 1;

                 byte targetPosition[4];
                 byte targetVelocity[4];

                 targetPosition[0] = dataIn[1];
                 targetPosition[1] = dataIn[2];
                 targetPosition[2] = dataIn[3];
                 targetPosition[3] = dataIn[4];

                 targetVelocity[0] = dataIn[5];
                 targetVelocity[1] = dataIn[6];
                 targetVelocity[2] = dataIn[7];
                 targetVelocity[3] = dataIn[8];

                 steps[ID] = *((long*)targetPosition);
                 stepsSec[ID] = *((float*)targetVelocity);

                 debugPrint(MSG_STATES,steps[ID],ID);
                 sendResponseMsg(cmdID,0,0);
            }
            case UPDATE_STEPPERS:
            {
                byte countsSec1[4];
                byte countsSec2[4];
                byte countsSec3[4];
                byte countsSec4[4];
                byte countsSec5[4];
                byte countsSec6[4];

                countsSec1[0] = dataIn[0];
                countsSec1[1] = dataIn[1];
                countsSec1[2] = dataIn[2];
                countsSec1[3] = dataIn[3];

                countsSec2[0] = dataIn[4];
                countsSec2[1] = dataIn[5];
                countsSec2[2] = dataIn[6];
                countsSec2[3] = dataIn[7];

                countsSec3[0] = dataIn[8];
                countsSec3[1] = dataIn[9];
                countsSec3[2] = dataIn[10];
                countsSec3[3] = dataIn[11];

                countsSec4[0] = dataIn[12];
                countsSec4[1] = dataIn[13];
                countsSec4[2] = dataIn[14];
                countsSec4[3] = dataIn[15];

                countsSec5[0] = dataIn[16];
                countsSec5[1] = dataIn[17];
                countsSec5[2] = dataIn[18];
                countsSec5[3] = dataIn[19];

                countsSec6[0] = dataIn[20];
                countsSec6[1] = dataIn[21];
                countsSec6[2] = dataIn[22];
                countsSec6[3] = dataIn[23];

                sPointer[0]->setSpeed(*((float*)(countsSec1)));
                sPointer[1]->setSpeed(*((float*)(countsSec2)));
                sPointer[2]->setSpeed(*((float*)(countsSec3)));
                sPointer[3]->setSpeed(*((float*)(countsSec4)));
                sPointer[4]->setSpeed(*((float*)(countsSec5)));
                sPointer[5]->setSpeed(*((float*)(countsSec6)));

                sendResponseMsg(cmdID,0,0);
                break;
            }
            case UPDATE_STATES:
            {
                 posMode = 1;

                 byte targetPosition1[4];
                 byte targetVelocity1[4];
                 byte targetPosition2[4];
                 byte targetVelocity2[4];
                 byte targetPosition3[4];
                 byte targetVelocity3[4];
                 byte targetPosition4[4];
                 byte targetVelocity4[4];
                 byte targetPosition5[4];
                 byte targetVelocity5[4];
                 byte targetPosition6[4];
                 byte targetVelocity6[4];


                 targetPosition1[0] = dataIn[0];
                 targetPosition1[1] = dataIn[1];
                 targetPosition1[2] = dataIn[2];
                 targetPosition1[3] = dataIn[3];
                 targetPosition2[0] = dataIn[4];
                 targetPosition2[1] = dataIn[5];
                 targetPosition2[2] = dataIn[6];
                 targetPosition2[3] = dataIn[7];
                 targetPosition3[0] = dataIn[8];
                 targetPosition3[1] = dataIn[9];
                 targetPosition3[2] = dataIn[10];
                 targetPosition3[3] = dataIn[11];
                 targetPosition4[0] = dataIn[12];
                 targetPosition4[1] = dataIn[13];
                 targetPosition4[2] = dataIn[14];
                 targetPosition4[3] = dataIn[15];
                 targetPosition5[0] = dataIn[16];
                 targetPosition5[1] = dataIn[17];
                 targetPosition5[2] = dataIn[18];
                 targetPosition5[3] = dataIn[19];
                 targetPosition6[0] = dataIn[20];
                 targetPosition6[1] = dataIn[21];
                 targetPosition6[2] = dataIn[22];
                 targetPosition6[3] = dataIn[23];


                 targetVelocity1[0] = dataIn[24];
                 targetVelocity1[1] = dataIn[25];
                 targetVelocity1[2] = dataIn[26];
                 targetVelocity1[3] = dataIn[27];
                 targetVelocity2[0] = dataIn[28];
                 targetVelocity2[1] = dataIn[29];
                 targetVelocity2[2] = dataIn[30];
                 targetVelocity2[3] = dataIn[31];
                 targetVelocity3[0] = dataIn[32];
                 targetVelocity3[1] = dataIn[33];
                 targetVelocity3[2] = dataIn[34];
                 targetVelocity3[3] = dataIn[35];
                 targetVelocity4[0] = dataIn[36];
                 targetVelocity4[1] = dataIn[37];
                 targetVelocity4[2] = dataIn[38];
                 targetVelocity4[3] = dataIn[39];
                 targetVelocity5[0] = dataIn[40];
                 targetVelocity5[1] = dataIn[41];
                 targetVelocity5[2] = dataIn[42];
                 targetVelocity5[3] = dataIn[43];
                 targetVelocity6[0] = dataIn[44];
                 targetVelocity6[1] = dataIn[45];
                 targetVelocity6[2] = dataIn[46];
                 targetVelocity6[3] = dataIn[47];

                 steps[0] = *((long*)targetPosition1);
                 stepsSec[0] = *((float*)targetVelocity1);
                 steps[1] = *((long*)targetPosition2);
                 stepsSec[1] = *((float*)targetVelocity2);
                 steps[2] = *((long*)targetPosition3);
                 stepsSec[2] = *((float*)targetVelocity3);
                 steps[3] = *((long*)targetPosition4);
                 stepsSec[3] = *((float*)targetVelocity4);
                 steps[4] = *((long*)targetPosition5);
                 stepsSec[4] = *((float*)targetVelocity5);
                 steps[5] = *((long*)targetPosition6);
                 stepsSec[5] = *((float*)targetVelocity6);

                 sendResponseMsg(cmdID,0,0);
                 break;
            }
            case CALIBRATE:
            {
                for(int i = 0;i < 6;i++)
                {
                  // Set current position in Motor frame
                  sPointer[i]->setCurrentPosition((long)(limits[i]*stepsDeg[i]));

                  // Check for reversed action
                  if ((i == 1) || (i == 2) || (i == 4))
                    // If reversed, must transform to Joint frame
                    steps[i] = -(long)(limits[i]*stepsDeg[i]);
                  else
                    steps[i] = (long)(limits[i]*stepsDeg[i]);

                  stepsSec[i] = 0.0;
                }

                sendResponseMsg(cmdID,0,0);
                break;
            }
            case READ:
            {
                // Returns position in Motor frame
                byte ID = dataIn[0];
                int32_t position = sPointer[ID]->currentPosition();

                byte result[4];
                result[0] = (position & 0x000000ff);
                result[1] = (position & 0x0000ff00) >> 8;
                result[2] = (position & 0x00ff0000) >> 16;
                result[3] = (position & 0xff000000) >> 24;

                sendResponseMsg(cmdID,result,4);
                break;
            }
            case READ_ALL:
            {
                int32_t position[6];
                byte result[4*MAX_STEPPERS];
                
                for(int i = 0;i < MAX_STEPPERS;i++)
                {
                    position[i] = sPointer[i]->currentPosition();
                    
                    result[4*i] = (position[i] & 0x000000ff);
                    result[4*i+1] = (position[i] & 0x0000ff00) >> 8;
                    result[4*i+2] = (position[i] & 0x00ff0000) >> 16;
                    result[4*i+3] = (position[i] & 0xff000000) >> 24;
                }
                
                sendResponseMsg(cmdID,result,4*MAX_STEPPERS);          
            }
            case SET:
            {
              byte newPosition1[4];
              byte newPosition2[4];
              byte newPosition3[4];
              byte newPosition4[4];
              byte newPosition5[4];
              byte newPosition6[4];

              newPosition1[0] = dataIn[0];
              newPosition1[1] = dataIn[1];
              newPosition1[2] = dataIn[2];
              newPosition1[3] = dataIn[3];
              newPosition2[0] = dataIn[4];
              newPosition2[1] = dataIn[5];
              newPosition2[2] = dataIn[6];
              newPosition2[3] = dataIn[7];
              newPosition3[0] = dataIn[8];
              newPosition3[1] = dataIn[9];
              newPosition3[2] = dataIn[10];
              newPosition3[3] = dataIn[11];
              newPosition4[0] = dataIn[12];
              newPosition4[1] = dataIn[13];
              newPosition4[2] = dataIn[14];
              newPosition4[3] = dataIn[15];
              newPosition5[0] = dataIn[16];
              newPosition5[1] = dataIn[17];
              newPosition5[2] = dataIn[18];
              newPosition5[3] = dataIn[19];
              newPosition6[0] = dataIn[20];
              newPosition6[1] = dataIn[21];
              newPosition6[2] = dataIn[22];
              newPosition6[3] = dataIn[23];

              // In Motor frame
              sPointer[0]->setCurrentPosition(*((long*)newPosition1));
              // Reversed
              sPointer[1]->setCurrentPosition(-*((long*)newPosition2));
              // Reversed
              sPointer[2]->setCurrentPosition(-*((long*)newPosition3));
              sPointer[3]->setCurrentPosition(*((long*)newPosition4));
              // Reversed
              sPointer[4]->setCurrentPosition(-*((long*)newPosition5));
              sPointer[5]->setCurrentPosition(*((long*)newPosition6));

              // In Joint frame
              steps[0] = *((long*)newPosition1);
              // Reversed
              steps[1] = *((long*)newPosition2);
              // Reversed
              steps[2] = *((long*)newPosition3);
              steps[3] = *((long*)newPosition4);
              // Reversed
              steps[4] = *((long*)newPosition5);
              steps[5] = *((long*)newPosition6);

              sendResponseMsg(cmdID,0,0);
              break;
            }
            default:
            {
                break;
            }
        }
    }
};

#endif
