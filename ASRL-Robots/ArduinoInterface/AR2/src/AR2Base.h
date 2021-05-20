
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
#define AR2STEPPER_CREATE 0x00
#define AR2STEPPER_READ 0x01
#define AR2STEPPER_SET_STATES 0x02
#define AR2_READ 0x03
#define AR2_SET_STATES 0x04
#define AR2_RESET 0x05

class Stepper : public LibraryBase
{
    public:

    AccelStepper* sPointer[MAX_STEPPERS];

    // Expressed in Joint frame
    long steps[6] = {0,0,0,0,0,0};
    float stepsSec[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

    // Transformation from Joint frame to Motor frame or vice verse
    byte transforms[6] = {1,-1,-1,1,-1,1};

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
            for (byte i = 0;i < 6;i++)
            {
              if(sPointer[i] != NULL)
              {
                if(transforms[i])
                {
                  sPointer[i]->moveTo(steps[0]);
                  sPointer[i]->setSpeed(stepsSec[0]);
                  sPointer[i]->runSpeedToPosition();
                }
                else
                {
                  sPointer[i]->moveTo(-steps[0]);
                  sPointer[i]->setSpeed(-stepsSec[0]);
                  sPointer[i]->runSpeedToPosition();
                }
              }
            }
    }

    public:

    void commandHandler(byte cmdID,byte* dataIn,unsigned int payload_size)
    {
        switch (cmdID)
        {
            case AR2STEPPER_CREATE:
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
            case AR2STEPPER_READ:
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
            case AR2STEPPER_SET_STATES:
            {
                 byte ID = dataIn[0];
                 posMode = 1;

                 byte targetPosition[4];
                 byte targetVelocity[4];
                 for(int i = 0;i < 4,i++)
                 {
                     targetPosition[i] = dataIn[i+1];
                     targetVelocity[i] = dataIn[i+5];
                 }

                 steps[ID] = *((long*)targetPosition);
                 stepsSec[ID] = *((float*)targetVelocity);

                 debugPrint(MSG_STATES,steps[ID],ID);
                 sendResponseMsg(cmdID,0,0);
                 break;
            }
            case AR2_READ:
            {
                int32_t position[MAX_STEPPERS];
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
                break;
            }
            case AR2_SET_STATES:
            {
                 posMode = 1;

                 byte targetPosition[6][4];
                 byte targetVelocity[6][4];

                 for (int i = 0;i < 6;i++)
                 {
                   for (int j = 0;j < 4;j++)
                   {
                     targetPosition[i][j] = dataIn[4*i+j];
                     targetVelocity[i][j] = dataIn[4*i+j+24];
                   }
                 }

                 for (int i = 0;i < 6;i++)
                 {
                   steps[i] = *(long*)targetPosition[i]
                   stepsSec[i] = *((float*)targetVelocity[i]);
                 }

                 sendResponseMsg(cmdID,0,0);
                 break;
            }
            case AR2_RESET:
            {

              byte newPosition[MAX_STEPPERS][4];

              // In joint frame
              for (int i = 0;i < 6; i++)
              {
                for (int j = 0;j < 6; j++)
                {
                  newPosition[i][j] = dataIn[4*i+j];
                }
              }

              for (int i = 0;i < 6; i++)
              {
                // In motor frame
                if (transforms[i] == 1)
                  sPointer[i]->setCurrentPosition(*((long*)newPosition[i]));
                else
                  sPointer[i]->setCurrentPosition(-*((long*)newPosition[i]));

                // In Joint frame
                steps[i] = *((long*)newPosition[i]);
              }

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
