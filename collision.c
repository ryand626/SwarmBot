#include <TimerOne.h>
 
// Motor constants
 
// THESE ONES ARE PINS
#define MTRLFWD 6 
#define MTRLREV 7
#define MTRRFWD 8
#define MTRRREV 9
#define MTRLEN 14
#define MTRREN 15
 
// JUST CONSTANTS
#define STOPPED 0
#define FORWARD 1
#define ROTATEL 2
#define ROTATER 3
#define FSWINGR 4 // forward swing right
#define FSWINGL 5
#define RSWINGR 6 // reverse swing right
#define RSWINGL 7
#define REVERSE 8
 
#define SLOW 60 
#define MEDIUM 125
#define FAST 255
 
// State Constants
#define COUNTER_LENGTH 10
#define MAX_RUNTIME 5000
//Movement Types
#define STRAIGHT 1
#define RIGHT 2
#define LEFT 3
#define HALT 6
 
// Collision Pads
#define PAD1 22
#define PAD2 23
#define PAD3 24
#define PAD4 25
 
#define PAD1_STATE2 10
#define PAD2_STATE2 11
#define PAD3_STATE2 12
#define PAD4_STATE2 13
 
// State variables
int counter;
int state;
 
// Motor Functions
void setMotors(int,int);
 
// Color Sensor Functions
int colorValue();
void colorMeasure();
 
// State functions
void move(int state);
int checkstate(int state);
 
// Collision variables
int previousState;
int collisionTimer;
int collisonTimeout;
bool collide;
 
void setup() {
  //motor setup
  pinMode(MTRLFWD,OUTPUT);
  pinMode(MTRRFWD,OUTPUT);
  pinMode(MTRLREV,OUTPUT);
  pinMode(MTRRREV,OUTPUT);
  pinMode(MTRLEN,OUTPUT);
  pinMode(MTRREN,OUTPUT);
  digitalWrite(MTRLEN, LOW);
  digitalWrite(MTRREN, LOW);
 
  // Initial motor state
  state = HALT;
 
  // Collision setup
  collisionTimer = 0;
  collisonTimeout = 0;
  collide = false;
  attachInterrupt(1,collision, RISING);
}
 
void loop() {
  //don't let it run forever

  // If colliding, reposition
  if(collide == true){
    reposition();
  // If not colliding, move according to state
  }else{
    switch (state) {
      case FORWARD:
        setMotors(FORWARD,SLOW);
        break;
      case RIGHT:
        setMotors(FSWINGR,SLOW);
        break;
      case LEFT:
        setMotors(FSWINGL,SLOW);
        break;
      default:
        setMotors(STOPPED,SLOW);
        break;
    } 
  }
}
 
 
// Set direction, velocity
void setMotors(int i, int v) {
 switch(i) {
   case FORWARD:
     analogWrite(MTRLFWD,v);
     analogWrite(MTRLREV,0);
     analogWrite(MTRRFWD,v);
     analogWrite(MTRRREV,0);
     digitalWrite(MTRREN,HIGH);
     digitalWrite(MTRLEN,HIGH);
     break;
   case ROTATEL:
     analogWrite(MTRLFWD,0);
     analogWrite(MTRLREV,v);
     analogWrite(MTRRFWD,v);
     analogWrite(MTRRREV,0);
     digitalWrite(MTRREN,HIGH);
     digitalWrite(MTRLEN,HIGH);
     break;
   case ROTATER:
     analogWrite(MTRLFWD,v);
     analogWrite(MTRLREV,0);
     analogWrite(MTRRFWD,0);
     analogWrite(MTRRREV,v);
     digitalWrite(MTRREN,HIGH);
     digitalWrite(MTRLEN,HIGH);
     break;
   case FSWINGL:
     analogWrite(MTRLFWD,0);
     analogWrite(MTRLREV,0);
     analogWrite(MTRRFWD,v);
     analogWrite(MTRRREV,0);
     digitalWrite(MTRREN,HIGH);
     digitalWrite(MTRLEN,HIGH);
     break;
   case FSWINGR:
     analogWrite(MTRLFWD,v);
     analogWrite(MTRLREV,0);
     analogWrite(MTRRFWD,0);
     analogWrite(MTRRREV,0);
     digitalWrite(MTRREN,HIGH);
     digitalWrite(MTRLEN,HIGH);
     break;
   case RSWINGL:
     analogWrite(MTRLFWD,0);
     analogWrite(MTRLREV,v);
     analogWrite(MTRRFWD,0);
     analogWrite(MTRRREV,0);
     digitalWrite(MTRREN,HIGH);
     digitalWrite(MTRLEN,HIGH);
     break;
   case RSWINGR:
     analogWrite(MTRLFWD,0);
     analogWrite(MTRLREV,0);
     analogWrite(MTRRFWD,0);
     analogWrite(MTRRREV,v);
     digitalWrite(MTRREN,HIGH);
     digitalWrite(MTRLEN,HIGH);
     break;
   case REVERSE:
     analogWrite(MTRLFWD,0);
     analogWrite(MTRLREV,v);
     analogWrite(MTRRFWD,0);
     analogWrite(MTRRREV,v);
     digitalWrite(MTRREN,HIGH);
     digitalWrite(MTRLEN,HIGH);
     break;
   default: //includes "STOP" case
     analogWrite(MTRLFWD,0);
     analogWrite(MTRLREV,0);
     analogWrite(MTRRFWD,0);
     analogWrite(MTRRREV,0);
     digitalWrite(MTRLEN,LOW);
     digitalWrite(MTRLEN,LOW);
     break;
  }
}
 
// YOU DON COLLID
void collision(){
  // Store previous state
  if(!collide){
    previousState = state;
  }
  // Set current state and know that you collided
  state = checkPads();
  collide = true;
  // Set the timeout of the collision movement to 500ms from now
  collisonTimeout = 500;
  // Set the current time of collision
  collisionTimer = millis();
  //communicateCollision();
}
 
// return the pad that got hit, starting the check at PAD1
// Eventually will have different timeouts here
int checkPads(){
  for (int i = PAD1; i < PAD4; i++){
    if(digitalRead(i) == HIGH){
      return i;
    }
  }
  return 0;
}
 
// checks to see if it's been enough time since the last collision
bool timeOut(){
  if(millis() > collisionTimer + collisonTimeout){
   return true;
  } else{
    return false;
  }
}
 
// Say where you were hit
void communicateCollision(){
}
 
// reposition according to pad that was hit, and the last time you were hit
// ACTUALLY MOVE ACCORDINGLY
void reposition(){
    switch(state){  
      case PAD1: 
        setMotors(REVERSE,MEDIUM);
        if(timeOut()){
          collisonTimeout = 500;
          collisionTimer = millis();
          state = PAD1_STATE2;
        }
        break;
      case PAD1_STATE2:
        setMotors(FSWINGR,MEDIUM);
        if(timeOut()){
          endCollide();
        }
        break;  
      case PAD2:
        setMotors(FSWINGR, MEDIUM);
        if(timeOut()){
          collisonTimeout = 500;
          collisionTimer = millis();
          state = PAD2_STATE2;
        }
        break;
      case PAD2_STATE2:
        endCollide();
        break;
      case PAD3:
        setMotors(ROTATEL, MEDIUM);
        if(timeOut()){
          collisonTimeout = 500;
          collisionTimer = millis();
          state = PAD3_STATE2;
        }
        break;
      case PAD3_STATE2:
        endCollide();
        break;
      case PAD4:
        setMotors(FSWINGR, MEDIUM);
        if(timeOut()){
          collisonTimeout = 500;
          collisionTimer = millis();
          state = PAD4_STATE2;
        }
        break;
      case PAD4_STATE2:
        endCollide();
        break;  
      default:
        endCollide();
        break;
    }
}
 
void endCollide(){
  state = previousState;
  collide = false;
}
