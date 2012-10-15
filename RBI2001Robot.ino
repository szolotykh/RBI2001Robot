#include <BluetoothClient.h>
#include <Servo.h>
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
#define DOWN_MAX 980
#define UP_MAX 65
Servo neck;
int neckPos;
int neckStage = DOWN;

//============ MAP =====================
int stage = 100;
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
//0x08 to 0x0F Reserved  
//0x10 and higher User defined

int lineCount = 0;

int detectLine = 0;

// ============ Light sensors ==============
#define LIGHT_MEAN 300

int leftLight; // A2
int centerLight; // A1
int rightLight; // A0
int frontLight; // A3

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
  
  // Mouth
  pinMode(MOUTH_TOUCH_PIN, INPUT);
  mouth.attach(MOUTH_MOTOR_PIN);
  
  // neck
  neck.attach(NECK_MOTOR_PIN);
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
      setLeftMotor(0);
      setRightMotor(90);
    }else{
      if(leftLight <= LIGHT_MEAN && rightLight >= LIGHT_MEAN){
        setLeftMotor(90);
        setRightMotor(0);
      }else{
          moveRobot(100);
      }
    }
}

void loop(){
  // Read sensors
  leftLight = analogRead(A2);
  centerLight = analogRead(A1);
  rightLight = analogRead(A0);
  frontLight = analogRead(A3);
  // Read neck potentiometr
  neckPos = analogRead(NECK_POT_PIN);
  // Read touch
  touch = digitalRead(6);
  
  // Main switch
  
  switch(stage){
  case 100: // Move forword and rotate right
    moveRobot(100);
    delay(3000);
    rotateRight(100);
    delay(3600);
    stage = 101;
  break;
  case 101: //find 3th line
    moveRobot(100);
    if(findLine(centerLight, 3)==1)
      stage = 102;
  break;
  case 102:
    rotateLeft(100);
    if(findLine(frontLight, 1)==1)
      stage = 103;
  break;
  case 103:
    followLine();
    if(findLine(centerLight, 1)==1)
      stage = 104;
  break;
  case 104:
    rotateLeft(100);
    if(findLine(frontLight, 2)==1)
      stage = 105;
  break;
  case 105:
    followLine();
    if(findLine(centerLight, 3)==1){
        moveRobot(-100);
        delay(150);
        stage = 106;
        mouth.write(IN);
        neck.write(DOWN);
    }
  break;
  case 106:
    stopRobot();
    if(neckStage == DOWN){
    if(neckPos > DOWN_MAX){
      neck.write(STOP);
    }
    if(touch == 0){
      neckStage = UP;
      neck.write(UP);
    }
  }else{
    if(neckPos < UP_MAX){
      neck.write(STOP);
      mouth.write(OUT);
      delay(3000);
      mouth.write(STOP);
    }  
  }
  break;
  /*
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
    if(isDark(frontLight)==0){
          detectLine = 1;
    }else{
      if(isDark(frontLight)==1 && detectLine==1){
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
      delay(2700);
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
    
      }
    }
  break;
  case 6:
  break;
  */
  }
  
  // === Display data ===
  Serial.print(leftLight);
  Serial.print(" - ");
  Serial.print(centerLight);
  Serial.print(" - ");
  Serial.print(rightLight);
  Serial.print(" - ");
  Serial.println(frontLight);
  Serial.println(stage);
  
  //delay(1);
}
