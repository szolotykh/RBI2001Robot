/*==================================
 10/10/2012 2:29AM
 RBE 2001
 Team #16
 Mouth test program
==================================*/
#include <Servo.h>

#define IN 0
#define OUT 180
#define STOP 90

Servo mouth; // Mouth motor
byte touch; // Value of touch sensor

void setup(){
  // Begin serial transmission
  Serial.begin(9600);
  // Setup mouth sensor
  pinMode(6, INPUT);
  // Setup mouth motor
  mouth.attach(12);
  // Activate mouth
  mouth.write(IN); // Rod IN
}

void loop(){
  // Read mouth touch sensor value
  touch = digitalRead(6);
  // Print mouth touch sensor value
  Serial.println(touch);
  if (touch == 0){ // Rod inside
    mouth.write(STOP); // Stop mouth
    delay(2000); // Wait
    mouth.write(OUT); // Rod OUT
    delay(3000); // Wait
    mouth.write(IN); //Rod IN
  }
  else{
    delay(15); // Small delay
  }
}
