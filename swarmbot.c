/*
===============================================================================
|     Tufts University EE31 Junior Design Project                             |
|     Created by Ryan Dougherty, Nick Andre, Bryan Zhang                      |
===============================================================================
*/

// Timer used for color sensor
#include <TimerOne.h>

//---------------------------- MOTOR CONSTANTS --------------------------------

// Motor pins
#define MTRLFWD 6 // if high -> left forward
#define MTRLREV 7 // if high -> left reverse
#define MTRRFWD 8 // if high -> right forward
#define MTRRREV 9 // if high -> right reverse
#define MTRLEN 14 // if high -> left motor is enabled
#define MTRREN 15 // if high -> right motor is enabled

// Motor configurations
#define STOPPED 0
#define FORWARD 1
#define ROTATEL 2
#define ROTATER 3
#define FSWINGR 4 // forward swing right
#define FSWINGL 5 // forward swing left
#define RSWINGR 6 // reverse swing right
#define RSWINGL 7 // reverse swing left
#define REVERSE 8

// Speed
#define SLOW 60 
#define MEDIUM 125
#define FAST 255

//--------------------------- SENSOR CONSTANTS --------------------------------
// Pins
#define REDLED 13
#define BLUELED 12
#define LIGHTSENSOR A0

// States........used?
#define BLUESTATE 0
#define REDSTATE 1

// Constants
#define DARK_THRESHOLD 120
//#define BLUE_DIFFERENCE 50
#define DARK 0
#define RED 1
#define BLUE 2

// State Constants
#define COUNTER_LENGTH 10
#define MAX_RUNTIME 5000
//------------------------ COLLISION CONSTANTS --------------------------------

#define BUMPER_DEBOUNCE_MILLIS 500

// Collision Pads
#define PAD0 22
#define PAD1 23
#define PAD2 24
#define PAD3 25
#define PAD4 26
#define PAD5 27
 
#define PAD0_STATE2 28
#define PAD1_STATE2 29
#define PAD2_STATE2 30
#define PAD3_STATE2 31
#define PAD4_STATE2 32
#define PAD5_STATE2 33


// Searching states
#define GO_UNTIL_COLLIDE 0
#define HUNTING_FOR_COLOR 1 //searching
#define FOUND_RED 2 
#define FOUND_BLUE 3
#define LLOST_BLUE 4 // JUST LOST BLUE, TURNING LEFT 
#define RLOST_BLUE 5   // Failed to find blue turning left, turning right instead
#define LLOST_RED 6
#define RLOST_RED 7
#define TURN_AROUND_RED 9
#define TURN_AROUND_BLUE 10
#define HALT 8

//--------------------------- Color variables ---------------------------------
int colorState;
int currentColorMeasurement, lastColorMeasurement;
int blueValue, redValue;
int lastColorReturned;

//------------------------- Collision variables -------------------------------
volatile int previousState;
volatile int collisionTimer;
volatile int collisionTimeout;
volatile bool collide;
volatile int numInts = 0;
volatile int debounceTimer = 0;
//--------------------------- State variables ---------------------------------
int counter;
int rotateTime = 200;
volatile int state;
//--------------------------- Motor functions ---------------------------------
void setMotors(int,int);
//--------------------------- Color functions ---------------------------------
int colorValue();
void colorMeasure();
//--------------------------- State functions ---------------------------------
void move();
void checkstate();



// ============================ Initialization ================================
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
  pinMode(REDLED,OUTPUT);
  pinMode(BLUELED,OUTPUT);
  digitalWrite(REDLED, HIGH);
  digitalWrite(BLUELED,LOW);
  colorState = REDSTATE;
  lastColorMeasurement = DARK;
  currentColorMeasurement = DARK;
  //Interrupt Setup for color
  //Timer1.initialize(10000); //10 milliseconds
  //Timer1.attachInterrupt(colorMeasure);

  // set initial state to hunting
  state = HALT;
  previousState = HALT;

  // Collision setup
  debounceTimer = 0;
  collisionTimer = 0;
  collisionTimeout = 0;
  collide = false;
  attachInterrupt(2,collision, RISING); // pin 21s


  
  //Serial.begin(9600);
}

// ============================ Main Loop =====================================
void loop() {
  // While not in the halting state, update the state with the sensors
  // then move according to state
  int color = colorValue();
  
  // If colliding, reposition
  if(collide == true && state != HALT){
    reposition();
  } 

  // If not colliding, move according to state
  else{
    //checkstate();
    move();
  }

  // Don't go forever
  if (millis() > MAX_RUNTIME){
    //state = HALT;
  }

  Serial.print("current state: ");Serial.println(state);
  //delay(1000);
}

//============================= Color Sensor ==================================

// Return current color value
int colorValue() {
  if (lastColorMeasurement != currentColorMeasurement) {
    return DARK;
  } else if (currentColorMeasurement == RED) {
    return RED;
  } else {
    return BLUE;
  }
}

//Take measurement (ISR)
void colorMeasure() {
  if (colorState == BLUESTATE) {
    lastColorMeasurement = currentColorMeasurement;
    blueValue = analogRead(LIGHTSENSOR);
    if (blueValue < DARK_THRESHOLD && redValue < DARK_THRESHOLD) {
      currentColorMeasurement = DARK;
    } else if (blueValue > DARK_THRESHOLD && redValue > DARK_THRESHOLD) {
      currentColorMeasurement = RED;
    } else {
      currentColorMeasurement = BLUE;
    }
    digitalWrite(REDLED,HIGH);
    digitalWrite(BLUELED,LOW);
    colorState = REDSTATE;
  } else {
    redValue = analogRead(LIGHTSENSOR);
    digitalWrite(REDLED,LOW);
    digitalWrite(BLUELED,HIGH);
    colorState = BLUESTATE;
  } 
}

//============================ Movement Logic =================================

// Updates the state of the machine according to the color sensor
void checkstate(){
  switch (state){
    case GO_UNTIL_COLLIDE:
      //only transitions out via interrupt
      break;
    case HUNTING_FOR_COLOR:
      if(colorValue() == RED){
        counter = 0;
        state = FOUND_RED;
      } else if(colorValue() == BLUE){
        state = FOUND_BLUE;
      } else{
        state = HUNTING_FOR_COLOR;
      }
      break;
    case FOUND_RED: //Should this really have a timeout?
      if(colorValue() == RED){
        state = FOUND_RED;
      }else{
        counter = 0;
        state = LLOST_RED;
      }
      break;

    // FOUND BLUE
    case FOUND_BLUE: //3
      if(colorValue() == BLUE){
        state = FOUND_BLUE;
      } else {
        counter = 0;
        state = LLOST_BLUE;
      }
      break;

    // JUST LOST BLUE, TURNING LEFT 
    case LLOST_BLUE:
      if(colorValue() == BLUE){
        state = FOUND_BLUE;
      }else if(counter < COUNTER_LENGTH){
        counter ++;
        state = LLOST_BLUE;
      }else{
        counter = 0;
        state = RLOST_BLUE;
      }
      break;

    // Failed to find blue turning left, turning right instead
    case RLOST_BLUE:
      if(colorValue() == BLUE){
        state = FOUND_BLUE;
      } else if(counter < COUNTER_LENGTH * 2){
        counter ++;
        state = RLOST_BLUE;
      }else{
        state = HALT;
      }
      break;

      case LLOST_RED:
      if(colorValue() == RED){
        state = FOUND_RED;
      }else if(counter < COUNTER_LENGTH){
        counter ++;
        state = LLOST_RED;
      }else{
        counter = 0;
        state = RLOST_RED;
      }
      break;

    // Failed to find RED turning left, turning right instead
    case RLOST_RED:
      if(colorValue() == RED){
        state = FOUND_RED;
      } else if(counter < COUNTER_LENGTH * 2){
        counter ++;
        state = RLOST_RED;
      }else{
        state = HALT;
      }
      break;
    case TURN_AROUND_BLUE:
      if(counter > rotateTime){
        state = LLOST_BLUE;
      }
      counter++;
      break;
    case TURN_AROUND_RED:
      if(counter > rotateTime){
        state = LLOST_RED;
      }
      counter++;
      break;
    // Halting case
    default:
      state = HALT;
      break;
  }
}

// Moves the swarmbot according to the state
void move(){
  switch (state){
    case GO_UNTIL_COLLIDE:
      setMotors(FORWARD,FAST);
      break;
    case HUNTING_FOR_COLOR: 
      setMotors(FORWARD,FAST);
      break;
    case FOUND_RED:
      setMotors(FORWARD,SLOW);
      break;
    case FOUND_BLUE:
      setMotors(FORWARD, SLOW);
      break;
    case LLOST_BLUE:
      setMotors(ROTATEL, SLOW);
      break;
    case RLOST_BLUE:
      setMotors(ROTATER,SLOW);
      break;
    case LLOST_RED:
      setMotors(ROTATEL, SLOW);
      break;
    case RLOST_RED:
      setMotors(ROTATER,SLOW);
      break;
    case TURN_AROUND_BLUE:
      setMotors(ROTATER, FAST);
      break;
    case TURN_AROUND_RED:
      setMotors(ROTATEL, FAST);
      break;
    // covers halt
    default:
      setMotors(STOPPED, 0);
      break;
  }
}

//=========================== Motor Settings ==================================
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

// ========================= Collision Detection ==============================

// Collision Interrupt Service Routine
void collision(){
  // For debounce
  unsigned long now = millis();
  if ((now - debounceTimer) < BUMPER_DEBOUNCE_MILLIS) return;
  debounceTimer = now;


  // Store previous state if not already colliding
  if(!collide){
    previousState = state;
    // Set the timeout of the collision movement to 500ms from now
    collisionTimeout = 500;
    // Set the current time of collision
    collisionTimer = millis();
  }
  // Set current state and know that you collided
  state = checkPads();
  // Make sure a pad was actually hit
  collide = true;

  //communicateCollision(); // <-will add this in later once collision is figured out
  
}
 
// return the pad that got hit, starting the check at PAD1
// Eventually will have different timeouts here
int checkPads(){
  for (int i = PAD0; i < PAD5; i++){
    if(digitalRead(i) == HIGH){
      return i;
    }
  }

  // default
  return PAD0;
}
 
// checks to see if it's been enough time since the last collision
bool timeOut(){
  if(millis() > collisionTimer + collisionTimeout){
   return true;
  } else{
    return false;
  }
}
 
// Say where you were hit
void communicateCollision(){
}
 
// reposition according to pad that was hit, and the last time you were hit
void reposition(){
    // Serial.print("reposition, state: ");Serial.println(state);
    // Serial.print("numInts:"); Serial.println(numInts);
    // Serial.print("debounceTimer: ");Serial.println(debounceTimer);
    switch(state){  
      case PAD0: 
        setMotors(REVERSE,MEDIUM);
        if(timeOut()){
          collisionTimeout = 500;
          collisionTimer = millis();
          state = PAD0_STATE2;
        }
        break;

      case PAD0_STATE2:
        if(previousState == GO_UNTIL_COLLIDE){
          setMotors(ROTATER, FAST);
        }else if(previousState == FOUND_RED){
          setMotors(ROTATEL, FAST);
        }else if(previousState == FOUND_BLUE){
          setMotors(ROTATER, FAST);
        }else{
          setMotors(FSWINGR,MEDIUM);
        }
        if(timeOut()){
          if(previousState == GO_UNTIL_COLLIDE){
            state = HUNTING_FOR_COLOR;
            collide = false;
          }else if(previousState == FOUND_RED){
            state = TURN_AROUND_RED;
            counter = 0;
            collide = false;
          }else if(previousState == FOUND_BLUE){
            state = TURN_AROUND_BLUE;
            counter = 0;
            collide = false;
          }else{
            endCollide();
          }
          setMotors(STOPPED,0);
        }
        break;  

      case PAD1: 
        setMotors(REVERSE,MEDIUM);
        if(timeOut()){
          collisionTimeout = 500;
          collisionTimer = millis();
          state = PAD1_STATE2;
        }
        break;

      case PAD1_STATE2:
        setMotors(ROTATER,MEDIUM);
        if(timeOut()){
          endCollide();
        }
        break; 

      case PAD2:
        setMotors(REVERSE, MEDIUM);
        if(timeOut()){
          collisionTimeout = 500;
          collisionTimer = millis();
          state = PAD2_STATE2;
        }
        break;

      case PAD2_STATE2:
        setMotors(ROTATEL,MEDIUM);
        if(timeOut()){
          endCollide();
        }
        break;

      case PAD3:
        setMotors(STOPPED, 0);
        if(timeOut()){
          collisionTimeout = 500;
          collisionTimer = millis();
          state = PAD3_STATE2;
        }
        break;

      case PAD3_STATE2:
        endCollide();
        break;

      case PAD4:
        setMotors(STOPPED, 0);
        if(timeOut()){
          collisionTimeout = 500;
          collisionTimer = millis();
          state = PAD4_STATE2;
        }
        break;

      case PAD4_STATE2:
        endCollide();
        break; 

      case PAD5:
        setMotors(STOPPED,0);
        if(timeOut()){
          collisionTimeout = 500;
          collisionTimer = millis();
          state = PAD5_STATE2;
        }
        break;

      case PAD5_STATE2:
        if(timeOut()){
          endCollide();
        }
        break;

      default:
        endCollide();
        state = HALT;
        break;
    }
}
// Go back to state before first collision, exit collision states 
void endCollide(){
  collide = false;
  setMotors(STOPPED,0);
  state = previousState;
}