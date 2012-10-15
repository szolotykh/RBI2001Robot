#include <BluetoothClient.h>
#include <Servo.h>
#include <TimerOne.h>

unsigned long int endTime=0;

// =========== LEDs ===================

void setUpLED(){
  pinMode(22, OUTPUT);
  pinMode(32, OUTPUT);
}

void onLED(){
  digitalWrite(22, HIGH);
  digitalWrite(32, HIGH);
}
void offLED(){
  digitalWrite(22, LOW);
  digitalWrite(32, LOW);
}
// =========== Bluetooth ==============
int spentRod = 4;
int newRod = 1;

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

byte msg[6];

byte mType;
byte mLenght;
byte mSourse;
byte mDist;
byte mData;
byte mCheck;

byte spentRods[4];
byte newRods[4];

byte fSpentFuelRod = 0;
byte fNewFuelRod = 0;

byte waitByte(){
  byte inByte;
  while(1){
    if(Serial3.available() > 0){
      inByte = Serial3.read();
      //Serial.println(inByte, HEX);
      break;
    }
  }
  return inByte;
}

byte receiveMsg(){
  if (waitByte() == 0x5F){
            Serial.println("msg");
             mLenght = waitByte();
             mType = waitByte();
             mSourse = waitByte();
             mDist = waitByte();
             if(mLenght == 6)
               mData = waitByte();
             else
               mData = 0;
             mCheck = waitByte();
             if(255-(mLenght+mType+mSourse+mDist+mData) != mCheck)
               return 0;
        }
        return 1;
}

byte sentHeartbeat(){
  byte msg[]={0x5F,0x06,HEARTBEAT,0x01,0x00,0x00,0xF1};
  Serial3.write(msg, 7);
  return 1;
}
byte sentSpentFuelRod(){
  byte msg[]={0x5F,0x06,RAD_ALERT,0x2C,0x00,0x00,0xCA};
  Serial3.write(msg, 7);
  return 1;
}
byte sentNewFuelRod(){
  byte msg[]={0x5F,0x06,RAD_ALERT,0xFF,0x00,0x00,0xF7}; // <====?
  Serial3.write(msg, 7);
  return 1;
}
void receiveRodsInf(){
  
  while(1){
    if(receiveMsg()==1){
          if(mType == 0x01){
               //Serial.println("n1");
               spentRods[0] = mData & 1;
               spentRods[1] = (mData>>1) & 1;
               spentRods[2] = (mData>>2) & 1;
               spentRods[3] = (mData>>3) & 1;
               for(byte i=0;i<4;i++){
                 if(spentRods[i]==0){
                   spentRod = i+1;
                 }
               }
               break;
            }
        }
  }
  while(1){
    if(receiveMsg()==1){
      if(mType == 0x02){
              //Serial.println("n2");
               newRods[0] = (mData>>3) & 1;
               newRods[1] = (mData>>2) & 1;
               newRods[2] = (mData>>1) & 1;
               newRods[3] = mData & 1;
               for(byte i=0;i<4;i++){
                 if(newRods[i]==1){
                   newRod = i+1;
                 }
               }
               break;
      }
    }
   }

//newRod = 1;
//spentRod = 3;
}

void timerIsr()
{
  sentHeartbeat();
  if(fSpentFuelRod==1)
    sentSpentFuelRod();
  if(fNewFuelRod==1)
    sentNewFuelRod();
}

// =========== Mouth ===================
#define MOUTH_MOTOR_PIN 12
#define MOUTH_TOUCH_PIN 6
#define IN 0
#define OUT 180
#define STOP 90
Servo mouth;
byte touch;

// =========== Neck ====================
#define NECK_MOTOR_PIN 7
#define NECK_POT_PIN A11
#define DOWN 150
#define UP 0
#define DOWN_MAX 1000 //980 before
#define UP_MAX 65
Servo neck;
int neckPos;
int neckStage = DOWN;

void neckDown(){
  neck.write(DOWN);
  while(1){
      neckPos = analogRead(NECK_POT_PIN);
      if(neckPos > DOWN_MAX){
        neck.write(STOP);
        break;
      }
  }
}

void neckUp(){
  neck.write(UP);
  while(1){
      neckPos = analogRead(NECK_POT_PIN);
      if(neckPos < UP_MAX){
        neck.write(STOP);
        break;
      }
  }
}

//============ MAP =====================
int lineCount = 0;
int detectLine = 0;

// ============ Light sensors ==============
#define LIGHT_MEAN 300

int leftLight; // A2
int centerLight; // A1
int rightLight; // A0
int frontLight; // A3
int frontLight2;

int isBright(int sensor_val){
  if(sensor_val<=LIGHT_MEAN)
    return 1;
  else
    return 0;
}

int isDark(int sensor_val){
  if(sensor_val>LIGHT_MEAN)
    return 1;
  else
    return 0;
}

// ================ Motors ==================
Servo leftMotor;
Servo rightMotor;

int LeftMotorSpeed = 100;
int RightMotorSpeed = 100;


void setLeftMotor(int sp){
  leftMotor.write(map(sp,-100,100,0,180));
}

void setRightMotor(int sp){
  rightMotor.write(map(sp,-100, 100, 180, 0));
}

void rotateLeft(int sp){
  setLeftMotor(-sp);
  setRightMotor(sp);
}

void rotateRight(int sp){
  setLeftMotor(sp);
  setRightMotor(-sp);
}
void moveRobot(int sp){
  setLeftMotor(sp);
  setRightMotor(sp);
}
void stopRobot(){
  setLeftMotor(0);
  setRightMotor(0);
}

byte findLine(int sensorValue, int lineInd){
// Find line event
    if(isDark(sensorValue)==1){
      if(detectLine==0){
        detectLine = 1;
        lineCount++;
      }
      if(lineCount==lineInd){
          lineCount = 0;
          detectLine = 0;
          return 1;
      }
    }else{
      detectLine = 0;
    }
    return 0;
}

void followLine(){
   // Follow line alghorithm
    if(leftLight >= LIGHT_MEAN && rightLight <= LIGHT_MEAN){
      setLeftMotor(-20);
      setRightMotor(100);
    }else{
      if(leftLight <= LIGHT_MEAN && rightLight >= LIGHT_MEAN){
        setLeftMotor(100);
        setRightMotor(-20);
      }else{
          moveRobot(100);
      }
    }
}

byte moveToLine(byte sp, byte ln){
  endTime = millis()+1000;
  centerLight = analogRead(A1);
  if(isDark(centerLight) == 1)
    ln++;
  while(1){
    leftLight = analogRead(A2);
    rightLight = analogRead(A0);
    followLine();
    if(millis() > endTime){
      centerLight = analogRead(A1);
      if(findLine(centerLight, ln)==1)
        break;
    }
  }
  return 1;
}
      
byte rotateLeftToLine(byte sp, byte ln){
  frontLight = analogRead(A3);
  if(isDark(frontLight) == 1)
    ln++;
  rotateLeft(sp);
  while(1){
      frontLight = analogRead(A3);
      if(findLine(frontLight, ln)==1){
        break;
      }
  }
  return 1;
}

byte rotateRightToLine(byte sp, byte ln){
  frontLight = analogRead(A3);
  if(isDark(frontLight) == 1)
    ln++;
  rotateRight(sp);
  while(1){
      frontLight = analogRead(A3);
      if(findLine(frontLight, ln)==1)
        break;
  }
  return 1;
}

// ================ SetUp =================
void setup(){
  Serial.begin(9600);
  leftMotor.attach(8);
  rightMotor.attach(9);
  
  // Mouth
  pinMode(MOUTH_TOUCH_PIN, INPUT);
  mouth.attach(MOUTH_MOTOR_PIN);
  
  // neck
  neck.attach(NECK_MOTOR_PIN);
  
  // Bluetooth
  Serial3.begin(115200);
  Timer1.initialize(1500000);
  Timer1.attachInterrupt( timerIsr );
  
  // LEDs
  setUpLED();
  
   //Bamp
  pinMode(31, INPUT);
  onLED(); 
  receiveRodsInf();
  offLED();
  setLeftMotor(LeftMotorSpeed);
  setRightMotor(RightMotorSpeed);
}

void loop(){
  // Intro
    moveRobot(100);
    delay(3000);
    rotateRight(100);
    delay(3500);
   
  // Stage 1
  moveRobot(100);
  while(1){  
    centerLight = analogRead(A1);
    if(findLine(centerLight, 3)==1)
      break;
  }
  rotateLeftToLine(100, 1);
  moveToLine(100, 1);
  rotateLeftToLine(100, 1);
  moveToLine(100, 3);
  
  moveRobot(-100);
  delay(150);
  stopRobot();
  
  mouth.write(IN);
  neck.write(DOWN);

  endTime = millis() + 6000;
  while(1){
    // Read neck potentiometr
    neckPos = analogRead(NECK_POT_PIN);
    // Read touch
    touch = digitalRead(6);
    
    if(neckStage == DOWN){
      if(neckPos > DOWN_MAX){
        neck.write(STOP);
      }
      if(touch == 0){
        mouth.write(STOP);
        neckStage = UP;
        neck.write(UP);
      }
      if(millis()>endTime){      
        neckUp();
        endTime = millis() + 6000;
        neck.write(DOWN);
      }
    }else{
      if(neckPos < UP_MAX){
        break;
      }
    }
  }
  
  fSpentFuelRod = 1;
  onLED();
  
  neck.write(STOP);
  moveRobot(-100);
  delay(1000);
        
  rotateLeftToLine(100, 1);
  moveToLine(100, spentRod); //find spent rod line
  rotateRightToLine(100, 1);
 
 // Move to spent rod
  endTime = millis()+1000; 
  while(1){
    leftLight = analogRead(A2);
    rightLight = analogRead(A0);
    frontLight2 = analogRead(A10);
    
    followLine();
    if(millis() > endTime){
      if(findLine(frontLight2, 1)==1)
        break;
    }
  }
  stopRobot();
  // Put spent rod
  mouth.write(OUT);
  delay(2500);
  mouth.write(STOP);
  fSpentFuelRod=0;
  offLED();
  
  // A littel bit back  
  moveRobot(-100);
  delay(1000);
  
  rotateRightToLine(100, 1);
  // Move to main line
  moveToLine(100, 1);
  
  if(spentRod < newRod){  
    rotateRightToLine(100, 1);
    moveToLine(100, newRod - spentRod);
    rotateLeftToLine(100, 1);
  }else{
    if(spentRod > newRod){ 
      rotateLeftToLine(100, 1);
      moveToLine(100, spentRod-newRod);//<<<<-
      rotateRightToLine(100, 1);
    }
  }

  // Move to new rod from main line
  endTime = millis()+1000;
  while(1){
    leftLight = analogRead(A2);
    rightLight = analogRead(A0);
    frontLight2 = analogRead(A10);
    
    followLine();
    if(millis()>endTime){
      if(findLine(frontLight2, 1)==1)
        break;
    }
  }
  
  mouth.write(IN);
  moveRobot(5);
  
  while(1){
    // Read touch
    touch = digitalRead(6);
    if(touch == 0){
      mouth.write(STOP);
      break;
    }
  }
  fNewFuelRod=1;
  onLED();
  // Go back for a little
  moveRobot(-100);
  delay(1000);
  // rotate on 180
  rotateRightToLine(100, 1);
  // Go to main line
  moveToLine(100, 1);
  
  rotateRightToLine(100, 1);
  if(newRod-1 != 0)
    moveToLine(100, newRod-1);
 
 while(1){
    leftLight = analogRead(A2);
    rightLight = analogRead(A0);
    followLine();
    if(digitalRead(31) == 0){
      break;
    }
 }
  
  stopRobot();
  neckDown();
  mouth.write(OUT);
  delay(2500);
  neckUp();
  mouth.write(STOP);
  fNewFuelRod=0;
  offLED();
 
  //=================== Stage 2 ======================
  Serial3.flush();
  for(int i = 0; i < 64; i++)
    Serial3.read();
  receiveRodsInf();
  
  moveRobot(-100);
  delay(1000);
  
  spentRod = 5 - spentRod;
  newRod = 5 - newRod;
  
  rotateLeftToLine(100, 1);
  moveToLine(100, 5);

  moveRobot(-100);
  delay(150);
  stopRobot();
  
  mouth.write(IN);
  neck.write(DOWN);
  neckStage = DOWN;
  
  endTime = millis() + 7000;
  while(1){
    // Read neck potentiometr
    neckPos = analogRead(NECK_POT_PIN);
    // Read touch
    touch = digitalRead(6);
    
    if(neckStage == DOWN){
      if(neckPos > DOWN_MAX){
        neck.write(STOP);
      }
      if(touch == 0){
        mouth.write(STOP);
        neckStage = UP;
        neck.write(UP);
      }
      if(millis()>endTime){
        //rotateRight(50);
        //delay(50);
        //rotateLeft(50);
        //delay(100);
        //rotateRight(50);
        //delay(50);
        
        neckUp();
        endTime = millis() + 7000;
        neck.write(DOWN);
      }
    }else{
      if(neckPos < UP_MAX){
        break;
      }
    }
  }
  fSpentFuelRod = 1;
  onLED();
  
  neck.write(STOP);
  moveRobot(-100);
  delay(1000);
        
  rotateLeftToLine(100, 1);
  moveToLine(100, spentRod);
  rotateLeftToLine(100, 1);
 
 // Move to spent rod
  endTime = millis()+1000; 
  while(1){
    leftLight = analogRead(A2);
    rightLight = analogRead(A0);
    frontLight2 = analogRead(A10);
    
    followLine();
    if(millis() > endTime){
      if(findLine(frontLight2, 1)==1)
        break;
    }
  }
  stopRobot();
  // Put spent rod
  mouth.write(OUT);
  delay(2500);
  mouth.write(STOP);
  fSpentFuelRod=0;
  offLED();
  
  // A littel bit back  
  moveRobot(-100);
  delay(1000);
  
  rotateRightToLine(100, 1);
  // Move to main line
  moveToLine(100, 1);
  
  if(spentRod < newRod){  
    rotateLeftToLine(100, 1);
    moveToLine(100, newRod - spentRod);
    rotateRightToLine(100, 1);
  }else{
    if(spentRod > newRod){ 
      rotateRightToLine(100, 1);
      moveToLine(100, spentRod-newRod);
      rotateLeftToLine(100, 1);
    }
  }

  // Move to new rod from main line
  endTime = millis()+1000;
  while(1){
    leftLight = analogRead(A2);
    rightLight = analogRead(A0);
    frontLight2 = analogRead(A10);
    
    followLine();
    if(millis()>endTime){
      if(findLine(frontLight2, 1)==1)
        break;
    }
  }
  
  mouth.write(IN);
  moveRobot(5);
  
  while(1){
    // Read touch
    touch = digitalRead(6);
    if(touch == 0){
      mouth.write(STOP);
      break;
    }
  }
  fNewFuelRod=1;
  onLED();
  // Go back for a littel
  moveRobot(-100);
  delay(1000);
  // rotate on 180
  rotateRightToLine(100, 1);
  // Go to main line
  moveToLine(100, 1);
  
  rotateLeftToLine(100, 1);
  if(newRod-1 != 0)
    moveToLine(100, newRod-1);
 
 while(1){
    leftLight = analogRead(A2);
    rightLight = analogRead(A0);
    followLine();
    if(digitalRead(31) == 0){
      break;
    }
 }
  
  stopRobot();
  neckDown();
  mouth.write(OUT);
  delay(2500);
  neckUp();
  mouth.write(STOP);
  fNewFuelRod=0;
  offLED();
  
  delay(15000);
  /*
  // === Display data ===
  Serial.print(leftLight);
  Serial.print(" - ");
  Serial.print(centerLight);
  Serial.print(" - ");
  Serial.print(rightLight);
  Serial.print(" - ");
  Serial.println(frontLight);
  */
  
  //delay(1);
}
