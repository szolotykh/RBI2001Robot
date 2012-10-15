// Robot Program
// Daniel Fox, Sergey Zolotykh
// RBE 2001 A02

#include <BluetoothClient.h>
#include <Servo.h>
#include <TimerOne.h>



// =========== LEDs ===================
// Setup LEDs
void setUpLED(){
  pinMode(22, OUTPUT);
  pinMode(32, OUTPUT);
}
// Turn on alert LEDs
void onLED(){
  digitalWrite(22, HIGH);
  digitalWrite(32, HIGH);
}
//Turn off alert LEDs
void offLED(){
  digitalWrite(22, LOW);
  digitalWrite(32, LOW);
}

// =========== Bluetooth ==============
int spentRod = 0; // Spent rod index
int newRod = 0; // New road index

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
byte mData; // Message data
byte mCheck; // Message check sum

byte spentRods[4]; //Spent rods array
byte newRods[4]; // New rods array

byte fSpentFuelRod = 0; // Spent rod on a field alert flag
byte fNewFuelRod = 0; // New rod on a field alert flag

// End time variable
unsigned long int endTime=0;

// Wait byte function
byte waitByte(){
  byte inByte; // incomming byte
  while(1){ // Wait for available byte
    if(Serial3.available() > 0){ // if byte is available
      inByte = Serial3.read(); //Read available byte
      // Print read byte
      //Serial.println(inByte, HEX);
      break;
    }
  }
  return inByte;
}
// Receive message function
byte receiveMsg(){
  if (waitByte() == 0x5F){ // Wait for start byte (0x5F)
             // Print indicater that message started receiving
             //Serial.println("msg");
             mLenght = waitByte(); // Message lenght
             mType = waitByte(); // Message type
             mSourse = waitByte(); // Message source address
             mDist = waitByte(); // Message destination address
             if(mLenght == 6) // if message has a data
               mData = waitByte(); // Read data
             else
               mData = 0;
             mCheck = waitByte(); //Message check sum
             // Check sum
             if(255-(mLenght+mType+mSourse+mDist+mData) != mCheck)
               return 0;
        }
        return 1;
}
// Sent heartbeat function
byte sentHeartbeat(){
  byte msg[]={0x5F,0x06,HEARTBEAT,0x01,0x00,0x00,0xF1};
  Serial3.write(msg, 7); // Write message
  return 1;
}
// Sent alert message 'Spent rode on field'
byte sentSpentFuelRod(){
  byte msg[]={0x5F,0x06,RAD_ALERT,0x2C,0x00,0x00,0xCA};
  Serial3.write(msg, 7); // Write message
  return 1;
}
// Sent alert message 'New rode on field'
byte sentNewFuelRod(){
  byte msg[]={0x5F,0x06,RAD_ALERT,0xFF,0x00,0x00,0xF7}; // <====?
  Serial3.write(msg, 7); // Write message
  return 1;
}
// Receive rods information

void receiveRodsInf(){
  while(1){
    if(receiveMsg()==1){ // Wait for message
          if(mType == 0x01){ // Spent rods
               spentRods[0] = mData & 1;
               spentRods[1] = (mData>>1) & 1;
               spentRods[2] = (mData>>2) & 1;
               spentRods[3] = (mData>>3) & 1;
               // Find available place for spent rod
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
    if(receiveMsg()==1){ // Wait for message
      if(mType == 0x02){ // New rods
               newRods[0] = (mData>>3) & 1;
               newRods[1] = (mData>>2) & 1;
               newRods[2] = (mData>>1) & 1;
               newRods[3] = mData & 1;
               // Find available new rod
               for(byte i=0;i<4;i++){
                 if(newRods[i]==1){
                   newRod = i+1;
                 }
               }
               break;
      }
    }
   }

//newRod = 1; // This was used for testing before Bluetooth was connected
//spentRod = 3; // This was used for testing before Bluetooth was connected
}

// Timer interrupt (1.5 sec)
void timerIsr(){
  sentHeartbeat(); // Sent heartbeat
  if(fSpentFuelRod==1)
    sentSpentFuelRod();
  if(fNewFuelRod==1)
    sentNewFuelRod();
}

// =========== Mouth ===================
#define MOUTH_MOTOR_PIN 12
#define MOUTH_TOUCH_PIN 6
#define IN 0 // Full speed into
#define OUT 180 // Full speed out
#define STOP 90 // Stop
Servo mouth; // Mouth motor
byte touch; // Mouth touch sensor value

// =========== Neck ====================
#define NECK_MOTOR_PIN 7
#define NECK_POT_PIN A11
#define DOWN 150
#define UP 0
#define DOWN_MAX 1000 //Potentiometer reading for neck down
#define UP_MAX 65  //Potentiometer reading for neck up
Servo neck; // Neck motor
int neckPos; // Neck potentiometer value
int neckStage = DOWN; // Neck stage

void neckDown(){ // Puts the neck downward
  neck.write(DOWN); // Neck down
  while(1){
      neckPos = analogRead(NECK_POT_PIN); // Read potentiometer value
      if(neckPos > DOWN_MAX){ // Neck reach down limit
        neck.write(STOP); // Stop neck
        break;
      }
  }
}

void neckUp(){ // Brings the neck up
  neck.write(UP); // Neck up
  while(1){
      neckPos = analogRead(NECK_POT_PIN); //potentiometer value
      if(neckPos < UP_MAX){ // Neck reach upper limit
        neck.write(STOP); // Stop neck
        break;
      }
  }
}

//============ MAP =====================
int lineCount = 0; // Line counter
int detectLine = 0; // Detect line flag

// ============ Light sensors ==============
#define LIGHT_MEAN 300 //Light mean

int leftLight; // Left light sensor value (A2)
int centerLight; // Center light sensor value (A1)
int rightLight; // Right light sensor value (A0)
int frontLight; // Front light sensor value (A3)
int frontLight2; // Second front light sensor value

// Light sensor value is bright, return 1 is it bright
int isBright(int sensor_val){
  if(sensor_val<=LIGHT_MEAN)
    return 1;
  else
    return 0;
}
// Light sensor value is dark, return 1 is it dark
int isDark(int sensor_val){
  if(sensor_val>LIGHT_MEAN)
    return 1;
  else
    return 0;
}

// ================ Motors ==================
Servo leftMotor; // Left motor
Servo rightMotor; // Right motor

// Set left motor speed from -100 to +100
void setLeftMotor(int sp){
  leftMotor.write(map(sp,-100,100,0,180));
}
// Set right motor speed from -100 to +100
void setRightMotor(int sp){
  rightMotor.write(map(sp,-100, 100, 180, 0));
}
// Rotate robot left with a speed sp, from -100 to +100
void rotateLeft(int sp){
  setLeftMotor(-sp);
  setRightMotor(sp);
}
// Rotate robot right with a speed sp, from -100 to +100
void rotateRight(int sp){
  setLeftMotor(sp);
  setRightMotor(-sp);
}
// Move robot with a speed sp, from -100 to +100
void moveRobot(int sp){
  setLeftMotor(sp);
  setRightMotor(sp);
}
// Stop robot
void stopRobot(){
  setLeftMotor(0);
  setRightMotor(0);
}
// Find line function
byte findLine(int sensorValue, int lineInd){
    // Find line events
    if(isDark(sensorValue)==1){ // if sensor on line
      if(detectLine==0){ // if sensor not on line
        detectLine = 1; // Line detected
        lineCount++; // Increase counter
      }
      if(lineCount==lineInd){
          lineCount = 0; // Reset counter
          detectLine = 0; // Reset detect line flag
          return 1;
      }
    }else{ // if sensor is not on line
      detectLine = 0; // Reset detect line flag
    }
    return 0;
}
// Follow the line function
void followLine(){
   // Follow line alghorithm
    if(leftLight >= LIGHT_MEAN && rightLight <= LIGHT_MEAN){
      // Move left
      setLeftMotor(-20);
      setRightMotor(100);
    }else{
      if(leftLight <= LIGHT_MEAN && rightLight >= LIGHT_MEAN){
        // Move right
        setLeftMotor(100);
        setRightMotor(-20);
      }else{
        // Move forward
        moveRobot(100);
      }
    }
}

// Move to line ln with a speed sp function
byte moveToLine(byte sp, byte ln){
  endTime = millis()+1000; // Set count delay
  centerLight = analogRead(A1); // Read sensor value
  if(isDark(centerLight) == 1) // if sensor on a line
    ln++; // Increase counter
  while(1){
    // Read follow line sensors
    leftLight = analogRead(A2);
    rightLight = analogRead(A0);
    followLine(); // Follow the line
    if(millis() > endTime){ // wait 1 sec and then start count
      centerLight = analogRead(A1); // Read sensor value
      if(findLine(centerLight, ln)==1) // Detect ln line
        break;
    }
  }
  return 1;
}
// Rotate robot left to line
byte rotateLeftToLine(byte sp, byte ln){
  frontLight = analogRead(A3); // Read sensor value
  if(isDark(frontLight) == 1) // if sensor on a line
    ln++; // Increase counter
  rotateLeft(sp); // Rotate robot
  while(1){
      frontLight = analogRead(A3); // Read sensor value
      if(findLine(frontLight, ln)==1){ // if ln line detected
        break;
      }
  }
  return 1;
}
// Rotate robot right to line ln
byte rotateRightToLine(byte sp, byte ln){
  frontLight = analogRead(A3); // Read sensor value
  if(isDark(frontLight) == 1) // if sensor on line
    ln++; // Increase counter
  rotateRight(sp); // Rotate robot
  while(1){
      frontLight = analogRead(A3); // Read sensor value
      if(findLine(frontLight, ln)==1) // If ln value detected
        break;
  }
  return 1;
}
// Grab spent rod function
void grabSpentRod(){
  mouth.write(IN); // Mouth IN mode
  neck.write(DOWN); // Start move neck down

  endTime = millis() + 6000; // Set timer event
  while(1){
    neckPos = analogRead(NECK_POT_PIN); // Read neck potentiometr
    touch = digitalRead(6); // Read touch sensor
    
    if(neckStage == DOWN){ // If neck is going down
      if(neckPos > DOWN_MAX){ // If neak reach down limit
        neck.write(STOP); // Stop neck
      }
      if(touch == 0){ // if rod in side the mouth
        mouth.write(STOP);// Stop mouth 
        neckStage = UP; // Neck into UP mode
        neck.write(UP); // Neck up
      }
      if(millis()>endTime){ // Reach timer event     
        neckUp(); // Neck up
        endTime = millis() + 6000; //Reset timer event
        neck.write(DOWN); // Neck down
      }
    }else{
      if(neckPos < UP_MAX){ // if neak reach upper limit
        break;
      }
    }
  }
  neck.write(STOP);// Stop neck
}
// Move to spent rods storage
void moveToSpentRodStorage(){
  endTime = millis()+1000;// set count delay
  while(1){
    // Read sensors values
    leftLight = analogRead(A2);
    rightLight = analogRead(A0);
    frontLight2 = analogRead(A10);
    
    followLine(); // Follow the line
    if(millis() > endTime){ // Wait 1 sec, then start count
      if(findLine(frontLight2, 1)==1) // Find first line
        break;
    }
  }
  stopRobot(); // Stop robot
}

// Move for new rod from main line
void moveForNewRod(){
  endTime = millis()+1000;// Setup count delay for 1 sec
  while(1){
    //read sensors values
    leftLight = analogRead(A2);
    rightLight = analogRead(A0);
    frontLight2 = analogRead(A10);
    
    followLine(); // Follow the line
    if(millis()>endTime){ // Wait 1 sec, then start count
      if(findLine(frontLight2, 1)==1) // Stop on first line
        break;
    }
  }
}

// Grab new rod function
void grabNewRod(){
  mouth.write(IN); // Set mouth IN
  moveRobot(5); // move robot slowly forward
  
  while(1){
    touch = digitalRead(6); // Read touch sensor value
    if(touch == 0){ // if mouth touch sensor is pressed
      mouth.write(STOP); // stop mouth motor
      break;
    }
  }
}

// Move to reactor function
void moveToReactor(){
  while(1){
      // Read sensors
      leftLight = analogRead(A2);
      rightLight = analogRead(A0);
      followLine();// Follow the line
      if(digitalRead(31) == 0){ // Wait for bump sensor
        break;
      }
  }
  stopRobot(); // Stop robot
}
// ================ SetUp =================
void setup(){
  // Begin serial transition
  Serial.begin(9600);
  // Attach motors
  leftMotor.attach(8);
  rightMotor.attach(9);
  
  // Setup mouth
  pinMode(MOUTH_TOUCH_PIN, INPUT);
  mouth.attach(MOUTH_MOTOR_PIN);
  
  // Attach neck motor
  neck.attach(NECK_MOTOR_PIN);
  
  //Setup bluetooth
  Serial3.begin(115200);
  Timer1.initialize(1500000);
  Timer1.attachInterrupt( timerIsr );
  
  //Setup LEDs
  setUpLED();
  
  //Setup bamp sensor
  pinMode(31, INPUT);
  // Receive information about field
  receiveRodsInf();
  // Move robot forward
  setLeftMotor(100);
  setRightMotor(100);
}
//==================== Main Loop ==============
void loop(){
  // Introduction Stage
  moveRobot(100); // Move robot forward
  delay(3000); // Wait 3 sec
  rotateRight(100); // Rotate robot right
  delay(3500); // Wait 3.5 sec
   
  // Stage 1
  // Move to spent rode
  moveRobot(100); // Move robot forward
  while(1){  
    centerLight = analogRead(A1); // Read sensor value
    if(findLine(centerLight, 3)==1) // Move to 3th line
      break;
  }
  rotateLeftToLine(100, 1); // Rotate robot left on 90 degree
  moveToLine(100, 1); // Move to main line
  rotateLeftToLine(100, 1); // Rotate robot on 90 degree(until it reach first line)
  moveToLine(100, 3); // Move to spent rode by main line
  
  moveRobot(-100); //move back a little bit
  delay(150);
  stopRobot();// Stop robot
  
  grabSpentRod(); // Grabbing spent rod
  
  // Start sending alert messages 'Spent rod on field' 
  fSpentFuelRod = 1;
  onLED(); // LEDs on
  
  // Move to storage of spent rods
  moveRobot(-100); // Move back for 1 sec
  delay(1000);
        
  rotateLeftToLine(100, 1); // ROtate left on 180 degrees
  moveToLine(100, spentRod); //find spent rod line by main line
  rotateRightToLine(100, 1); // rotate right on 90 degrees
 
 // Move to spent rod
 moveToSpentRodStorage();
 
  // Put spent rod
  mouth.write(OUT);
  delay(2500);
  mouth.write(STOP);
 
  fSpentFuelRod = 0; // Stop sent alert messages
  offLED(); // Turn LEDs off
  
  // A littel bit back  
  moveRobot(-100);
  delay(1000);
 
  rotateRightToLine(100, 1); // Rotate robot on 180 degrees
  moveToLine(100, 1); // Move to main line
  
  // Choosing path
  if(spentRod < newRod){  
    rotateRightToLine(100, 1); // Rotate right on 90 degrees
    moveToLine(100, newRod - spentRod); // To new rod line by main line
    rotateLeftToLine(100, 1); // Rotate left on 180 degrees 
  }else{
    if(spentRod > newRod){ 
      rotateLeftToLine(100, 1);// Rotate left on 180 degrees 
      moveToLine(100, spentRod-newRod); // To new rod line by main line
      rotateRightToLine(100, 1);// Rotate right on 90 degrees
    }
  }

  // Move for new rod from main line
  moveForNewRod();
  // Grabbing new rod
  grabNewRod();
  
  fNewFuelRod=1; // Start sent alert message 'New rod on field'
  onLED(); // Turn LED on
  
  // Go back for a little
  moveRobot(-100);
  delay(1000);
  
  rotateRightToLine(100, 1); // rotate on 180 degrees
  moveToLine(100, 1); // Go to main line
  
  rotateRightToLine(100, 1); // Rotate right on 90 degrees
  // Move forward
  if(newRod-1 != 0)
    moveToLine(100, newRod-1);
  // Move to first reactor
  moveToReactor();
  
  // Put new rod to reactor
  neckDown(); // Neck down
  mouth.write(OUT); // Mouth in OUT mode
  delay(2500); // Wait 2.5 sec
  neckUp(); // Neak up
  mouth.write(STOP); // Stop mouth
  fNewFuelRod=0; //Stop sent alert messages 'New rod on field'
  offLED(); // Turn of LEDs
 
  //=================== Stage 2 ======================
  // Flush serial buffers
  Serial3.flush();
  for(int i = 0; i < 64; i++)
    Serial3.read();
  // Receive new information about field
  receiveRodsInf();
  
  spentRod = 5 - spentRod; // Invert spent rod index
  newRod = 5 - newRod; // Invert spent rod index
  
  //Move to second reactor
  moveRobot(-100); // Move a little bit back for 1 sec
  delay(1000);
  
  rotateLeftToLine(100, 1); // Rotate robot left for 180 degrees
  moveToLine(100, 5); // Move forward to second reactor by main line

  moveRobot(-100); // Move back a little
  delay(150);
  stopRobot(); // Stop the robot
  
  grabSpentRod(); // Grabbing spent rod
  // Start sending alert messages 'Spent rod on field' 
  fSpentFuelRod = 1;
  onLED(); // LEDs on
  
  moveRobot(-100); // Move back for 1 sec
  delay(1000);
        
  rotateLeftToLine(100, 1); // Rotate robot left for 180 degrees
  moveToLine(100, spentRod); // Move robot to the spent rod line by main line
  rotateLeftToLine(100, 1); // Rotate robot left on 90 degrees
 
  // Move to spent rod
  moveToSpentRodStorage();
  // Put spent rod
  mouth.write(OUT); // Mouth in OUT mode
  delay(2500); // Wait 2.5 sec
  mouth.write(STOP); // Stop mouth motor
  // Start sent alert messages 'Spent rod on a field'
  fSpentFuelRod=0;
  offLED(); // Turn LCDs off
  
  // A littel bit back for 1 sec
  moveRobot(-100);
  delay(1000);
  
  rotateRightToLine(100, 1); // Rotate robot right on 180 degrees
  moveToLine(100, 1); // Move to main line
  
  // Choosing path
  if(spentRod < newRod){  
    rotateLeftToLine(100, 1); // Rotate robot left on 90 degrees
    moveToLine(100, newRod - spentRod); // Move to new rod line by main line
    rotateRightToLine(100, 1); // Rotate robot right on 90 degrees
  }else{
    if(spentRod > newRod){ 
      rotateRightToLine(100, 1); // Rotate robot right on 90 degrees
      moveToLine(100, spentRod-newRod); // Move to new rod line by main line
      rotateLeftToLine(100, 1); // Rotate robot left on 90 degrees
    }
  }

  // Move for new rod from main line
  moveForNewRod();
  // Grabbing new rod
  grabNewRod();
  
  //Start sent alert messages 'New rod on field'
  fNewFuelRod=1;
  onLED(); // Turn LEDs off
  // Go back for a littel
  moveRobot(-100);
  delay(1000);
  // rotate on 180
  rotateRightToLine(100, 1);
  // Go to main line
  moveToLine(100, 1);
  
  rotateLeftToLine(100, 1); // Rotate robot left on 90 degrees
  // Move robot to second reactor by main line
  if(newRod-1 != 0)
    moveToLine(100, newRod-1);
 
  // Move to second reactor
  moveToReactor();
  
  // Put rod to second reactor
  neckDown(); // Reck down
  mouth.write(OUT); // Mouth in OUT mode
  delay(2500); // Wait 2.5 sec
  neckUp(); // Neck up
  mouth.write(STOP); // Stop mouth motor
  //Stop sent alert messages 'New rod on field'
  fNewFuelRod=0;
  offLED();// Turn LEDs off
  
  delay(15000);// Wait 15 sec
}
