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
int collisionTimer;
int collisonTimeout;
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
 if (millis() > MAX_RUNTIME)
   state = HALT;
 switch (state) {
   // case FORWARD:
   //   setMotors(FORWARD,SLOW);
   //   if (color == BLUE) {
   //     state = RIGHT;
   //   }
   //   break;
   // case RIGHT:
   //   setMotors(FSWINGR,SLOW);
   //   if (color == DARK) {
   //     state = LEFT;
   //   }
   //   break;
   // case LEFT:
   //   setMotors(FSWINGL,SLOW);
   //   if (color == BLUE) {
   //     state = RIGHT;
   //   }
   //   break;
   default:
     setMotors(STOPPED,SLOW);
     break;
 }
 //State Architecture
 /*
 while (state != HALT){
   state = checkstate(state);
   move(state);
 }
 */


 /*
 //Challenge 1
 setMotors(FORWARD,MEDIUM);
 while (colorValue() != RED);
 setMotors(REVERSE,MEDIUM);
 delay(1000);
 setMotors(STOPPED,FAST);
 while(1);

 */
 /*
 //Challenge 2
 if (colorValue() == RED)
   setMotors(FORWARD,SLOW);
 else if (colorValue() == BLUE)
   setMOtors(STOPPED,SLOW);
 */
 
}

// Take color measurement
// void colorMeasure() {
//  if (colorState == BLUESTATE) {
//    lastColorMeasurement = currentColorMeasurement;
//    blueValue = analogRead(LIGHTSENSOR);
//    //Serial.print("Blue Reading: ");
//    //Serial.print(blueValue);
//    //Serial.println("");
//    if (blueValue < DARK_THRESHOLD && redValue < DARK_THRESHOLD) {
//      currentColorMeasurement = DARK;
//    } else if (blueValue <  redValue) {
//      currentColorMeasurement = RED;
//    } else {
//      currentColorMeasurement = BLUE;
//    }
//    digitalWrite(REDLED,HIGH);
//    digitalWrite(BLUELED,LOW);
//    colorState = REDSTATE;
//  } else {
//    redValue = analogRead(LIGHTSENSOR);
//    digitalWrite(REDLED,LOW);
//    digitalWrite(BLUELED,HIGH);
  
//    colorState = BLUESTATE;
//    lastColorReturned = DARK;
//  } 
// }

// // Return the current color seen by the sensor
// int colorValue() {
//  if (lastColorMeasurement != currentColorMeasurement) {
//    return lastColorReturned;
//  } else if (currentColorMeasurement == RED) {
//    return (lastColorReturned = RED);
//  } else if (currentColorMeasurement == BLUE) {
//    return (lastColorReturned = BLUE);
//  } else {
//    return (currentColorMeasurement = DARK);
//  }
// }

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
/*
// Updates the state of the machine according to the color sensor
int checkstate(int state){
 switch (state){
   // I SEE NOTHING->hunting for colors
   case HUNTING: //1
     if(colorValue() == RED){
       counter = 0;
       state = 2;
     } else if(colorValue() == BLUE){
       state = 3;
     } else{
       state = 1;
     }
     break;
   // TURN RIGHT THEN HALT
   case FOUND_RED: //2
     if(counter < COUNTER_LENGTH){
       state = 2;
       counter++;
     }else{
       state = HALT;
     }
     break;

   // FOUND BLUE
   case FOUND_BLUE: //3
     if(colorValue() == BLUE){
       state = 3;
     } else if(colorValue() == RED){
       counter = 0;
       state = 2;
     } else {
       counter = 0;
       state = 4;
     }

   // JUST LOST BLUE, TURNING LEFT 
   case 4:
     if(colorValue() == BLUE){
       state = 3;
     } else if(colorValue() == RED){
       counter = 0;
       state = 2;
     }else if(counter < COUNTER_LENGTH){
       counter ++;
       state = 4;
     }else{
       counter = 0;
       state = 5;
     }
     break;

   // Failed to find blue turning left, turning right instead
   case 5:
     if(colorValue() == BLUE){
       state = 3;
     }else if(colorValue() == RED){
       counter = 0;
       state = 2;
     } else if(counter < COUNTER_LENGTH * 2){
       counter ++;
       state = 5;
     }else{
       state = HALT;
     }
     break;

   // Halting case
   case default:
     state = HALT;
 }

 return state;
}

// Moves the swarmbot according to the state
void move(int state){
 switch{
   case 1: 
     setMotors(FORWARD,FAST);
     break;

   case 2:
     setMotors(ROTATER,FAST);
     break;

   case 3:
     setMotors(FORWARD, SLOW);
     break;

   case 4:
     setMotors(ROTATEL, SLOW);
     break;

   case 5:
     setMotors(ROTATER,SLOW);
     break;

   case default:
     setMotors(STOPPED, 0);
 }
}
*/

void collision(){
  collisionState = checkPads();
  //communicateCollision();
  if(checkLastCollision){
    collisionTimer = millis();
    reposition();
  }
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

  switch(collisionState){  
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
      setMotors(FSWINGL, MEDIUM);
      break;
  }
}

