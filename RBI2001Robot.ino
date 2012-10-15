#include <BluetoothClient.h>
#include <Servo.h>

unsigned long int endTime=0;

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

//============ MAP =====================
int stage = 108;
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
  while(1){
    leftLight = analogRead(A2);
    centerLight = analogRead(A1);
    rightLight = analogRead(A0);
    followLine();
    if(findLine(centerLight, ln)==1)
      break;
  }
  return 1;
}
      
byte rotateLeftToLine(byte sp, byte ln){
  rotateLeft(sp);
  while(1){
    frontLight = analogRead(A3);
    if(findLine(frontLight, ln)==1)
      break;
  }
  //delay(80);
  return 1;
}

byte rotateRightToLine(byte sp, byte ln){
  rotateRight(sp);
  while(1){
    frontLight = analogRead(A3);
    if(findLine(frontLight, ln)==1)
      break;
  }
  //delay(80);
  return 1;
}

void loop(){
  // Read sensors
  //leftLight = analogRead(A2);
  //centerLight = analogRead(A1);
  //rightLight = analogRead(A0);
  //frontLight = analogRead(A3);
  //frontLight2 = analogRead(A10);
  // Read neck potentiometr
  //neckPos = analogRead(NECK_POT_PIN);
  // Read touch
  //touch = digitalRead(6);
  
  /*
  // Intro
    moveRobot(100);
    delay(3000);
    rotateRight(100);
    delay(3500);
   */
  moveRobot(100);
  while(1){  
    centerLight = analogRead(A1);
    if(findLine(centerLight, 3)==1)
      break;
  }
  rotateLeftToLine(100, 1);
  moveToLine(100, 1);
  rotateLeftToLine(100, 2);
  moveToLine(100, 3);
  
  moveRobot(-100);
  delay(150);
  stopRobot();
  
  mouth.write(IN);
  neck.write(DOWN);

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
    }else{
      if(neckPos < UP_MAX){
        break;
      }
    }
  }
  neck.write(STOP);
  moveRobot(-100);
  delay(350);
        
  rotateLeftToLine(100, 3);
  moveToLine(100, 2);
  rotateRightToLine(100, 2);
 
 // Move to spent rod  
  while(1){
    leftLight = analogRead(A2);
    rightLight = analogRead(A0);
    frontLight2 = analogRead(A10);
    
    followLine();
    if(findLine(frontLight2, 1)==1)
      break;
  }
  stopRobot();
  // Put spent rod
  mouth.write(OUT);
  delay(2500);
  mouth.write(STOP);
  // A littel bit back
  
  
  moveRobot(-100);
  delay(1000);
  
  rotateRightToLine(100, 2);
  
  moveToLine(100, 1);
  rotateRightToLine(100, 2);
  moveToLine(100, 2);//<<<<-
  rotateLeftToLine(100, 2);
  
  // Move to new rod
  while(1){
    leftLight = analogRead(A2);
    rightLight = analogRead(A0);
    frontLight2 = analogRead(A10);
    
    followLine();
    if(findLine(frontLight2, 1)==1)
      break;
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
  
  // Go back for a littel
  moveRobot(-100);
  delay(400);
  // rotate on 180
  rotateRightToLine(100, 2);
  // Go to main line
  moveToLine(100, 1);
  
  //rotateRightToLine(75, 2);
  rotateLeftToLine(100, 4);
  moveToLine(100, 4); 
  stopRobot();
  
  moveRobot(-100);
  delay(75);
  stopRobot();
  
  neck.write(DOWN);
  while(1){
      neckPos = analogRead(NECK_POT_PIN);
      if(neckPos > DOWN_MAX){
        neck.write(STOP);
        break;
      }
  }
  mouth.write(OUT);
  delay(2500);
  
  neck.write(UP);
  while(1){
      neckPos = analogRead(NECK_POT_PIN);
      if(neckPos < UP_MAX){
        neck.write(STOP);
        break;
      }
  }
  
  
  delay(10000);
  
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
