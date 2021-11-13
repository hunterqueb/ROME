#include <AccelStepper.h>
#include <PacketSerial.h>

#define STEPPER_CREATE 0x01
#define SET_RADSEC 0x02
#define UPDATE_STEPPERS 0x03
#define SET_STATES 0x04
#define UPDATE_STATES 0x05
#define CALIBRATE 0x06
#define READ 0x07
#define SET 0x08
#define READ_ALL 0x09

PacketSerial packetizer;

// Number of steppers
int NUM = 6;
int posMode = 0;

// Expressed in Joint frame
long steps[6] = {0,0,0,0,0,0};
float stepsSec[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

// Steps per degree for each motor
float stepsDeg[6] = {1/(.022368421*2),1/.018082192,
                     1/.017834395,1/.021710526,
                     1/.045901639,1/.046792453};

// Limits of each joint, degrees from zero. Expressed in Motor frame.
// J2,J3,and J5 reversed here.
float limits[6] = {-170.0,132.0,-141.0,-155.0,105.0,-155.0};

// Assign pin numbers to stepper
byte pinNumbers[6][2] = {{2,3},{4,5},{6,7},{8,9},{10,11},{12,13}};

AccelStepper steppers[] = {AccelStepper(AccelStepper::DRIVER, pinNumbers[0][0], pinNumbers[0][1]),
                           AccelStepper(AccelStepper::DRIVER, pinNumbers[1][0], pinNumbers[1][1]),
                           AccelStepper(AccelStepper::DRIVER, pinNumbers[2][0], pinNumbers[2][1]),
                           AccelStepper(AccelStepper::DRIVER, pinNumbers[3][0], pinNumbers[3][1]),
                           AccelStepper(AccelStepper::DRIVER, pinNumbers[4][0], pinNumbers[4][1]),
                           AccelStepper(AccelStepper::DRIVER, pinNumbers[5][0], pinNumbers[5][1])};

void setup() 
{
  // Motor Setup
  int maxSpeed = 1000;
  
  for (int i = 0; i < 6; i++)
  {
    steppers[i].setMaxSpeed(1000);
    steppers[i].setSpeed(0);
  }

  // Packetizer Setup
  packetizer.begin(19200);
  packetizer.setPacketHandler(&onPacketReceived);
}

void loop() 
{
  packetizer.update();
  updateSteppers();

}

void onPacketReceived(const uint8_t* buffer, size_t size)
{
  // Get command ID, which is a single byte
  uint8_t CMD = buffer[0];

  switch(CMD)
  {
    case SET_STATES:
    {
      byte ID = buffer[1];
      posMode = 1;

      byte targetPos[4];
      byte targetVelocity[4];

      for (int i = 0; i < 4;i++)
      {
        targetPos[i] = buffer[i+2];
        targetVelocity[i] = buffer[i+2+4];
      }

      // Cast 4-byte array to long 
      steps[ID] = *((long*)targetPos);
      // Cast 4-byte array to float
      stepsSec[ID] = *((float*)targetVelocity);
      break;
    }
    
    case UPDATE_STATES:
    {
      posMode = 1;
      
      byte targetPositions[6][4];
      byte targetVelocities[6][4];

      // Read byte data from buffer
      for (int i = 0;i < 6;i++)
      {
        for (int j = 0; j < 4;j++)
        {
          int index = i*4 + j + 1;
          targetPositions[i][j] = buffer[index];
          targetVelocities[i][j] = buffer[index + 24];
        }
      }

      // Update current position and velocity commands
      for (int i = 0;i < 6;i++)
      {
        // Cast 4-byte array to long 
        steps[i] = *((long*)targetPositions[i]);
        // Cast 4-byte array to float
        stepsSec[i] = *((float*)targetVelocities[i]);
      }

      break;
    }
    case READ:
    {
      byte ID = buffer[1];
      int32_t count = steppers[ID].currentPosition();

      byte result[4];
      
      result[0] = (count & 0x000000ff);
      result[1] = (count & 0x0000ff00) >> 8;
      result[2] = (count & 0x00ff0000) >> 16;
      result[3] = (count & 0xff000000) >> 24;

      packetizer.send(result,sizeof(result));
      break;
    }
    case READ_ALL:
    {
      int32_t position[6];
      byte result[4*NUM];

      for (int i = 0;i < NUM;i++)
      {
        position[i] = steppers[i].currentPosition();

        result[4*i] = (position[i] & 0x000000ff);
        result[4*i+1] = (position[i] & 0x0000ff00) >> 8;
        result[4*i+2] = (position[i] & 0x00ff0000) >> 16;
        result[4*i+3] = (position[i] & 0xff000000) >> 24;
      }

      packetizer.send(result,sizeof(result));
      break;
    }
    case CALIBRATE:
    {
      for(int i = 0;i < 6;i++)
      {
        // Set current position in Motor frame
        steppers[i].setCurrentPosition((long)(limits[i]*stepsDeg[i]));

        // Check for reversed action
        if ((i == 1) || (i == 2) || (i == 4))
          // If reversed, must transform to Joint frame
          steps[i] = -(long)(limits[i]*stepsDeg[i]);
        else
          steps[i] = (long)(limits[i]*stepsDeg[i]);

        stepsSec[i] = 0.0;
      }
      break;
    }
    default:
    {
      // do nothing
      break;
    }
  }
}

void updateSteppers()
{
  int reversed;

  if (posMode = 1)
  {
    // Update steppers
    for (int i = 0; i < 6; i++)
    {
      if ((i == 1) || (i == 2) || (i == 4) || (i == 5))
        reversed = -1;
      else
        reversed = 1;
      
      steppers[i].moveTo(reversed * steps[i]);
      // NOTE: Not sure this does anything
      steppers[i].setSpeed(-stepsSec[i]);
      steppers[i].runSpeedToPosition();
    }
  }
  else
  {
    for (int i = 0;i < 6;i++)
      steppers[i].runSpeed();
  }
}
