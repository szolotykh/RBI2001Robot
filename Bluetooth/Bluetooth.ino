/*==================================
 10/10/2012 2:20AM
 RBE 2001
 Team #16
 Bluetooth test program
==================================*/
#include <TimerOne.h>

//0x00 Reserved  
#define STORAGE_AVEIL 0x01 //Storage tube availability
#define SUPPLY_AVEIL 0x02 //Supply tube availability
#define RAD_ALERT 0x03 //Radiation alert
#define STOP_MOVE 0x04 //Stop movement
#define RESUME_MOVE 0x05 //Resume movement 
#define ROBOT_STATIUS 0x06 // Robot status 
#define HEARTBEAT 0x07 //Heartbeat 
//0x08 to 0x0F Reserved  
//0x10 and higher User defined

int incomingByte = 0;   // for incoming serial data

byte mType; // Message type
byte mLenght; // Message lenght
byte mSourse; // Message source address
byte mDist; // Message destination address
byte mData; // Message data (1 byte)
byte mCheck; // Message check sum

byte spentRods[4]; // Spent rods array
byte newRods[4]; // New rods array

byte waitByte(){
  byte inByte;
  while(1){
    if(Serial3.available() > 0){
      // Read available byte
      inByte = Serial3.read();
      // Print each incomming byte
      //Serial.println(inByte, HEX);
      break;
    }
  }
  return inByte;
}

// Receive message
byte receiveMsg(){
  if (waitByte() == 0x5F){ // Wait for start byte
            // Print if message started receiving
            //Serial.println("msg");
             mLenght = waitByte();
             mType = waitByte();
             mSourse = waitByte();
             mDist = waitByte();
             if(mLenght == 6)
               // if lenght is 6, then message have 1 byte of data
               mData = waitByte();
             else
               mData = 0;
             // Check sum
             mCheck = waitByte();
             if(255-(mLenght+mType+mSourse+mDist+mData) != mCheck)
               return 0;
        }
        return 1;
}
// Sent heartbeat function
byte sentHeartbeat(){
  byte msg[]={0x5F,0x06,HEARTBEAT,0x01,0x00,0x00,0xF1};
  Serial3.write(msg, 7);
  return 1;
}
// Sent spent rod on a field function
byte sentSpentFuelRod(){
  byte msg[]={0x5F,0x06,RAD_ALERT,0x2C,0x00,0x00,0xCA};
  Serial3.write(msg, 7);
  return 1;
}
// Sent new rod on a field function
byte sentNewFuelRod(){
  byte msg[]={0x5F,0x06,RAD_ALERT,0xFF,0x00,0x00,0x09};
  Serial3.write(msg, 7);
  return 1;
}
// Timer interrupt
void timerIsr()
{
  sentHeartbeat();
}

void setup() {
        // Setup timer for 1.5 sec
        Timer1.initialize(1500000);
        Timer1.attachInterrupt( timerIsr );
        // Begin serial transmission
        Serial.begin(9600);
        Serial3.begin(115200);
        sentHeartbeat();
}

void loop() {
        // Wait for a message
        if(receiveMsg()==1){
          // Print type of message
          Serial.print("Type = ");
          Serial.println(mType);
          switch(mType){
            case 0x01:
               // Spent rods
               spentRods[0] = mData & 1;
               spentRods[1] = (mData>>1) & 1;
               spentRods[2] = (mData>>2) & 1;
               spentRods[3] = (mData>>3) & 1;
               // Print information about spent rods
               Serial.print("Spent Rods: ");
               for(byte i=0; i<4; i++){
                 Serial.print(spentRods[i]);
                 Serial.print(" ");
               }
               Serial.println(".");
            break;
            case 0x02:
               // New rods
               newRods[0] = mData & 1;
               newRods[1] = (mData>>1) & 1;
               newRods[2] = (mData>>2) & 1;
               newRods[3] = (mData>>3) & 1;
               // Print information about new rods
               Serial.print("New Rods: ");
               for(byte i=0; i<4; i++){
                 Serial.print(newRods[i]);
                 Serial.print(" ");
               }
               Serial.println(".");
            break;
          }
        }
}
