#include <PacketSerial.h>
#include <NewMotorLibrary.h>
#define command1 0x09


PacketSerial packetizer;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();


void setup() 
{
  packetizer.begin(115200);
  packetizer.setPacketHandler(&onPacketReceived);
}

void loop() 
{
  packetizer.update();
  
}

void onPacketReceived(const uint8_t* buffer, size_t size)
{
  // Get command ID, which is a single byte
  uint8_t ID = buffer[0];
  uint8_t dID = 0x09;
  
  if (ID == 0x09)
  {
    //uint8_t responseString[] = "Valid\n";
    //packetizer.send(responseString,sizeof(responseString));
    
    int numBytes = 4;
    uint8_t data[numBytes];
  
    for (int i = 0;i < 4; i++)
    {
      data[i] = buffer[i+1];
    }
  
    double decoded = *((double*)data);
    if (decoded == 32.43)
    {
      uint8_t responseString[] = "Victory\n";
      packetizer.send(responseString,sizeof(responseString));
    }
  }
  else
  {
    uint8_t responseString[] = "Bad command!\n";
    packetizer.send(responseString,sizeof(responseString));
    //packetizer.send(buffer,sizeof(buffer));
  }
}

