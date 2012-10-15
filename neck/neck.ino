/*==================================
 10/10/2012 2:39AM
 RBE 2001
 Team #16
 Neck test program
==================================*/
#include <Servo.h>

#define DOWN 180
#define UP 0
#define STOP 90

#define DOWN_MAX 980 // Down limit
#define UP_MAX 65 // Upper limit

Servo neck; // Neck motor
int neckPos; // Neck potentiometer sensor value
int neckStage = DOWN; // Neck stage (DOWN)

void setup(){
  Serial.begin(9600); // Begin serial transmission
  neck.attach(7); // Setup neck motor
  neck.write(DOWN); // Neak down
}

void loop(){
  // Read neck potentiometer sensor value
  neckPos = analogRead(A11);
  // Print neck potentiometer sensor value
  Serial.println(neckPos);
  if(neckStage == DOWN){ // if neck going down
    if(neckPos > DOWN_MAX){ // if neck reach down limit
      neck.write(STOP); // Stop neck
      delay(2000); // Wait
      neckStage = UP; // Set neck stage to UP
      neck.write(UP); // Neck UP
    }
  }else{
    if(neckPos < UP_MAX){ // If neck reach upper limit
      neck.write(STOP); // Stop neck
      delay(2000); // Wit
      neckStage = DOWN; // Set neck stage to DOWN
      neck.write(DOWN); // Neck DOWN
    }  
  }
  delay(50); // Small delay
}
