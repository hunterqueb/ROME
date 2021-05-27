#include <NewMotorLibrary.h>
#include <PacketSerial.h>

#define GET_COUNTS 0x00
#define GET_RAD 0x01
#define GET_RADSEC 0x02
#define GET_COUNTSSEC 0x03
#define SET_RADSEC 0x04
#define UPDATE_MOTORS 0x05
#define UPDATE_VOLTAGES 0x06

PacketSerial packetizer;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Initialize PID to inactive mode
int mode = 0;

// Initialize motor array
NewMotor motors[] = {NewMotor(1,&AFMS,2,23,3072,2.6,60.0,0.0,DIRECT), 
                       NewMotor(2,&AFMS,19,27,3072,2.6,60.0,0.0,DIRECT), 
                       NewMotor(3,&AFMS,18,25,3072,2.6,60.0,0.0,DIRECT)};

double Kp = 2.5,Ki = 60.25,Kd = 0.0;


void setup() 
{
  packetizer.begin(115200);
  packetizer.setPacketHandler(&onPacketReceived);
  AFMS.begin();

  for (int i = 0; i < 3; i++)
    motors[i].begin();

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() 
{
  // Receive new serial command packets
  packetizer.update();

  // Update motor PID's if in automatic mode 1
  if(mode == 1)
  {
        motors[0].updateMotor();
        motors[1].updateMotor();
        motors[2].updateMotor();
  }

//  if (packetizer.overflow())
//  {
//    digitalWrite(LED_BUILTIN, HIGH);
//  }

}

void onPacketReceived(const uint8_t* buffer, size_t size)
{
  // Get command ID, which is a single byte
  uint8_t CMD = buffer[0];

  switch(CMD)
  {
    case GET_COUNTS:
    {        
      byte ID = buffer[1];
    
      int32_t count = motors[ID-1].getCounts();
      byte result[4];
  
      result[0] = (count & 0x000000ff);
      result[1] = (count & 0x0000ff00) >> 8;
      result[2] = (count & 0x00ff0000) >> 16;
      result[3] = (count & 0xff000000) >> 24;
   
      packetizer.send(result,sizeof(result));
      break;
    }
    case UPDATE_VOLTAGES:
    {
      byte volt1[2];
      byte volt2[2];
      byte volt3[2];
                               
      for(int i=0;i<2;i++)
          volt1[i] = buffer[i];
      
      for(int i=2;i<4;i++)
          volt2[i-2] = buffer[i];
      
      for(int i=4;i<6;i++)
          volt3[i-4] = buffer[i];

      motors[0].setDuty(*((int*)(volt1)));
      motors[0].setDuty(*((int*)(volt2)));
      motors[0].setDuty(*((int*)(volt3)));
      
      break;
    }
    case SET_RADSEC:
    {
      mode = 1;
      byte ID = buffer[1];
      byte radSec[4];

      for (int i = 0;i < 4; i++)
        radSec[i] = buffer[i+2];  
      
      motors[ID].setSetpoint(*((float*)(radSec)));
      
      break;
    }
    case UPDATE_MOTORS:
    {
      mode = 1;
      
      byte radSec1[4];
      byte radSec2[4];
      byte radSec3[4];

      for (int i = 0;i < 4;i++)
      {
        radSec1[i] = buffer[i+1];
        radSec2[i] = buffer[i+5];
        radSec3[i] = buffer[i+9];
      }
      
      motors[0].setSetpoint(*((float*)(radSec1)));
      motors[1].setSetpoint(*((float*)(radSec2)));
      motors[2].setSetpoint(*((float*)(radSec3)));
      
      break;           
    }
    default:
    {
      uint8_t responseString[] = "Bad command!\n";
      packetizer.send(responseString,sizeof(responseString));
    }
  }
  
}
