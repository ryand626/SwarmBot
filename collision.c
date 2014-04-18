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

// Color Sensor Constants
//Pins
//#define REDLED 6
//#define BLUELED 7

#define LIGHTSENSOR A0

//States
#define BLUESTATE 0
#define REDSTATE 1

#define DARK_THRESHOLD 200

#define DARK 0
#define RED 1
#define BLUE 2

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



// Color variables
int colorState;
int currentColorMeasurement, lastColorMeasurement;
int blueValue, redValue;
int lastColorReturned;

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

int collisionState;


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
 collide = false;

 //Color Sensor Setup
 //pinMode(REDLED,OUTPUT);
 //pinMode(BLUELED,OUTPUT);
 
 //digitalWrite(REDLED, HIGH);
 //digitalWrite(BLUELED,LOW);
 
 //colorState = REDSTATE;
 
 //lastColorMeasurement = DARK;
 //lastColorReturned = DARK;
 
 //Interrupt Setup
 //Timer1.initialize(15000); //25 milliseconds
 //Timer1.attachInterrupt(colorMeasure);
 state = FORWARD;

 // Collision setup
 attachInterrupt(1,collision, RISING);
 collisionTimer = 0;
 collisonTimeout = 0;
}

void loop() {
 // While not in the halting state, update the state with the sensors
 // then move according to state
 //int color = colorValue();
  if (millis() > MAX_RUNTIME){
    state = HALT;
  }
  if(collide){
    reposition();
  }

 switch (state) {
   case FORWARD:
     setMotors(FORWARD,SLOW);
     // if (color == BLUE) {
     //   state = RIGHT;
     // }
     break;
   case RIGHT:
     setMotors(FSWINGR,SLOW);
     // if (color == DARK) {
     //   state = LEFT;
     // }
     break;
   case LEFT:
     setMotors(FSWINGL,SLOW);
     // if (color == BLUE) {
     //   state = RIGHT;
     // }
     break;
   default:
     setMotors(STOPPED,SLOW);
     break;
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
 state = i;
}

void collision(){
  previousState = state;
  state = checkPads();
  collide = true;
  //communicateCollision();
}

int checkPads(){
  for (int i = PAD1; i < PAD4; i++)
  {
    if(digitalRead(i) == HIGH){
      return i;
    }
  }
  return 0;
}

bool checkLastCollision(){
  // checks to see if it's been enough time since the last collision
  if(millis() > collisionTimer + collisonTimeout){
   return true;
  } else{
    return false;
  }
}

void communicateCollision(){
  // Say where you were hit
}

void reposition(){
  // reposition according to pad that was hit, and the last time you were hit
  // ACTUALLY MOVE ACCORDINGLY
  if(checkLastCollision){
    switch(state){  
      case PAD1: 
        setMotors(REVERSE,MEDIUM);
        delay(5000);
        setMotors(ROTATER,MEDIUM);
        break;
      case PAD2:
        setMotors(REVERSE, MEDIUM);
        delay(5000);
        setMotors(FSWINGR, MEDIUM);
        break;
      case PAD3:
        setMotors(REVERSE, MEDIUM);
        delay(5000);
        setMotors(ROTATEL, MEDIUM);
        break;
      case PAD4:
        setMotors(REVERSE, MEDIUM);
        delay(5000);
        setMotors(FSWINGR, MEDIUM);
        break;  
      default:
        break;
    }
    collisionTimer = millis();
  }
  collide = false;
}
