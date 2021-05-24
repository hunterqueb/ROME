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
NewMotor mPointer[] = {NewMotor(1,&AFMS,2,23,3072,2.6,60.0,0.0,DIRECT), 
                       NewMotor(2,&AFMS,19,27,3072,2.6,60.0,0.0,DIRECT), 
                       NewMotor(3,&AFMS,18,25,3072,2.6,60.0,0.0,DIRECT)};

void setup() 
{
  packetizer.begin(115200);
  packetizer.setPacketHandler(&onPacketReceived);
  AFMS.begin();

//  mPointer[0] =   NewMotor(0,&AFMS,2,23,3072,2.6,60.0,0.0,DIRECT);
//  mPointer[1] =   NewMotor(0,&AFMS,19,27,3072,2.6,60.0,0.0,DIRECT);
//  mPointer[2] =   NewMotor(0,&AFMS,18,25,3072,2.6,60.0,0.0,DIRECT);

  for (int i = 0; i < 3; i++)
    mPointer[i].begin();

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() 
{
  packetizer.update();

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
    
      int32_t count = mPointer[ID-1].getCounts();
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

      mPointer[0].setDuty(*((int*)(volt1)));
      mPointer[0].setDuty(*((int*)(volt2)));
      mPointer[0].setDuty(*((int*)(volt3)));
      
      break;
    }
    default:
    {
      uint8_t responseString[] = "Bad command!\n";
      packetizer.send(responseString,sizeof(responseString));
    }
  }
  
}
