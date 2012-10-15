#include <Servo.h>
#define NECK_MOTOR_PIN 7
#define NECK_POT_PIN A11
#define DOWN 150
#define UP 0
#define DOWN_MAX 980
#define UP_MAX 65
Servo neck;
int neckPos;
int neckStage = DOWN;

// Mouth
#define MOUTH_MOTOR_PIN 12
#define MOUTH_TOUCH_PIN 6
#define IN 0
#define OUT 180
#define STOP 90
Servo mouth;
byte touch;

void setup(){
  Serial.begin(9600);
  
  // Mouth
  pinMode(MOUTH_TOUCH_PIN, INPUT);
  mouth.attach(MOUTH_MOTOR_PIN);
  mouth.write(IN);
  
  // neck
  neck.attach(NECK_MOTOR_PIN);
  neck.write(DOWN);
}

void loop(){
  neckPos = analogRead(NECK_POT_PIN);
  touch = digitalRead(6);
  
  Serial.println(touch);
  //Serial.println(neckPos);
  
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
 
  delay(10);
}
