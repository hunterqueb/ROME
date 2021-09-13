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

// Expressed in Joint frame
long steps[6] = {0,0,0,0,0,0};
float stepsSec[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

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
    case UPDATE_STEPPERS:
    {
      byte targetPositions[6][4];
      byte targetVelocities[6][4];

      // Read byte data from buffer
      for (int i = 0;i < 6;i++)
      {
        for (int j = 0; j < 4;j++)
        {
          int index = i*4 + j + 1;
          targetPositions[i][j] = buffer[index];
          targetVelocities[i][j] = buffer[index + 25];
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
    default:
    {
      // do nothing
    }
  }
}

void updateSteppers()
{
  int reversed;
  
  // Update steppers
  for (int i = 0; i < 6; i++)
  {
    if (i == 1 || i == 2 || i ==4)
      reversed = -1;
    else
      reversed = 1;
    
    steppers[i].moveTo(reversed * steps[i]);
    steppers[i].setSpeed(stepsSec[i]);
    steppers[i].runSpeedToPosition();
  }
}
