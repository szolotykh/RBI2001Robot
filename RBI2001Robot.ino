#include <BluetoothClient.h>
#include <Servo.h>
//============ MAP =====================
int stage = 1;
int subStage = 1;
int spentRod = 3;
int newRod = 1;

//0x00 Reserved  
#define STORAGE_AVEIL 0x01 //Storage tube availability
#define SUPPLY_AVEIL 0x02 //Supply tube availability
#define RAD_ALERT 0x03 //Radiation alert
#define STOP_MOVE 0x04 //Stop movement
#define RESUME_MOVE 0x05 //Resume movement 
#define ROBOT_STATIUS 0x06 // Robot status 
#define HEARTBEAT 0x07 //Heartbeat 
//0x08 â€“ 0x0F Reserved  
//0x10 and higher User defined

int lineCount = 0;

int detectLine = 0;

// ============ Light sensors ==============
#define LIGHT_MEAN 300

int leftLight; // A2
int centerLight; // A1
int rightLight; // A0
int backLight; // A3

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
// ================ SetUp =================
void setup(){
  Serial.begin(9600);
  leftMotor.attach(8);
  rightMotor.attach(9);
  
  setLeftMotor(LeftMotorSpeed);
  setRightMotor(RightMotorSpeed);
}

void loop(){
  // Read sensors
  leftLight = analogRead(A2);
  centerLight = analogRead(A1);
  rightLight = analogRead(A0);
  backLight = analogRead(A3);
  // Main switch
  switch(stage){
  case 1:
    // Find line event
    if(isDark(centerLight)==1){
      if(detectLine==0){
        detectLine = 1;
        lineCount++;
      }
      if(lineCount==spentRod){
          stage = 2;
          detectLine=0;
      }
    }else{
      detectLine = 0;
    }
  break;
  case 2:// Rotation
    rotateRight(100);
    if(isDark(backLight)==0){
          detectLine = 1;
    }else{
      if(isDark(backLight)==1 && detectLine==1){
          detectLine=0;
          stage = 3;
      }
    }
  break;
  case 3:
  
   // Follow line alghorithm
    if(leftLight >= LIGHT_MEAN && rightLight <= LIGHT_MEAN){
      setLeftMotor(0);
      setRightMotor(100);
    }else{
      if(leftLight <= LIGHT_MEAN && rightLight >= LIGHT_MEAN){
        setLeftMotor(100);
        setRightMotor(0);
      }else{
        if(isDark(leftLight)&&isDark(rightLight)){
           stopRobot();
           stage = 4;
        }else{
          moveRobot(100);
        }
      }
    }
  break;
  case 4:
    stage = 5;
    subStage=1;
    detectLine = 0;
  break;
  case 5:
    if(subStage==1){
      moveRobot(-100);
      delay(1700);
      subStage = 2;
    }else{
      rotateRight(100);
      if(isDark(centerLight)==1 && detectLine==0){
        detectLine = 1;
      }
      if(isBright(centerLight)==1 && detectLine==1){
        //detectLine=2;
        detectLine=0;
        moveRobot(100);
        stage=6;
        subStage = 1;
    
      }/*
      if(isDark(centerLight)==1 && detectLine==2){
        detectLine=0;
        moveRobot(100);
        stage=6;
        subStage = 1;
      }*/
    }
  break;
  case 6:
  break;
  }
  
  // === Display data ===
  Serial.print(leftLight);
  Serial.print(" - ");
  Serial.print(centerLight);
  Serial.print(" - ");
  Serial.print(rightLight);
  Serial.print(" - ");
  Serial.println(backLight);
  Serial.println(stage);
  
  delay(10);
}
