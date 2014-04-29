/*
===============================================================================
|     Tufts University EE31 Junior Design Project                             |
|     Created by Ryan Dougherty, Nick Andre, Bryan Zhang                      |
|     Partner team: Brad Frizzell, Stephen Panaro, Cody Chen                  |
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
#define STOPPED 0 // stops
#define FORWARD 1 // go forward
#define ROTATEL 2 // rotate left
#define ROTATER 3 // rotate right
#define FSWINGR 4 // forward swing right
#define FSWINGL 5 // forward swing left
#define RSWINGR 6 // reverse swing right
#define RSWINGL 7 // reverse swing left
#define REVERSE 8 // go backward

#define MOTOR_OFFSET 0

// Speed
#define SLOW 60 
#define MEDIUM 75
#define FAST 80

//--------------------------- SENSOR CONSTANTS --------------------------------

// Pins
#define REDLED 13
#define BLUELED 12
#define LIGHTSENSOR A0

// Color States
#define BLUESTATE 0
#define REDSTATE 1

// Constants
#define BLUE_THRESHOLD 400  // Change this based on environment
#define BLUE_DIFFERENCE 450
#define RED_THRESHOLD 80
#define DARK 0
#define RED 1
#define BLUE 2

// State Constants
#define COUNTER_LENGTH 14
#define MAX_RUNTIME 5000

//------------------------ COLLISION CONSTANTS --------------------------------

#define BUMPER_DEBOUNCE_MILLIS 500
#define THIS_IS_MADNESS 100

// Collision Pads
#define PAD0 22 // 6
#define PAD1 23 // 8
#define PAD2 24 // 3
#define PAD3 25 // 7
#define PAD4 26 // 2
#define PAD5 27 // 4
 
#define PAD0_STATE2 28
#define PAD1_STATE2 29
#define PAD2_STATE2 30
#define PAD3_STATE2 31
#define PAD4_STATE2 32
#define PAD5_STATE2 33

#define PAD_HIT 34

// Searching states
#define GO_UNTIL_COLLIDE 0
#define HUNTING_FOR_COLOR 1
#define FOUND_RED 2
#define FOUND_BLUE 3
#define LLOST_BLUE 4 
#define RLOST_BLUE 5 
#define LLOST_RED 6
#define RLOST_RED 7
#define TURN_AROUND_RED 9
#define TURN_AROUND_BLUE 10
#define HALT 8

#define WAIT_UNTIL_COLOR 11
#define WAIT_UNTIL_DONE 12
#define FOUND_RED_FIRST 13
#define FOUND_BLUE_FIRST 14
#define COMMUNICATE_COLOR 15
#define DONESTATE 16
#define OVER 17
#define TURN_RED_RIGHT 18
#define TURN_BLUE_LEFT 19

//--------------------------- MUSIC CONSTANTS ---------------------------------

#define SPEAKER_PIN 11 // Speaker or Piezo buzzer on pin 11

#define NOTE_D6  1175
#define NOTE_F6  1397
#define NOTE_G6  1568
#define NOTE_A6  1760
#define NOTE_C7  2093

// -------------------------- COMM CONSTANTS -----------------------------------

#define MASTER_SLAVE_PIN 10
#define MASTER 0
#define SLAVE 1
#define maskPin 45
#define PACKET_MILLIS 500

// Colors
#define RED_COLOR 1
#define BLUE_COLOR 2

#define RED_PIN 32 // SET DESIRED PIN
#define BLUE_PIN 34// SET DESIRED PIN

// Our signal
const byte myBLUE_FOUND = 100; 
const byte myRED_FOUND = 109; 
const byte myDONE_MSG = 120; 
const byte myRECEIVED = 60;
const byte myNO_MESSAGE = 0;

// Companion signal
const byte BLUE_FOUND = 105;
const byte RED_FOUND = 114; 
const byte DONE_MSG = 125; 
const byte RECEIVED = 65;
const byte NO_MESSAGE = 5;

//--------------------------- Color variables ---------------------------------

int color;
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
volatile unsigned long now;
volatile unsigned long debounceTimer = 0;
int stopTheMadness;

//--------------------------- State variables ---------------------------------

int counter;
int rotateTime = 8;
volatile int state;
volatile bool masterSlave;
volatile bool startedSlave;

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
  // song setup
  pinMode(SPEAKER_PIN, OUTPUT);  

  // Comm Setup
  // Setup Clock for Carrier Frequency
  // see IR sketch for register explanations
  pinMode(5, OUTPUT);
  TCCR3A = _BV(COM3A0) | _BV(COM3B0) | _BV(WGM30) | _BV(WGM31); 
  TCCR3B = _BV(WGM32) | _BV(WGM33) | _BV(CS31);
  OCR3A = 39; // sets the value at which the register resets. 39 generates 25kHz
  digitalWrite(5, HIGH); // turn on the carrier

  // new comm things
  Serial3.begin(300);
  Serial.begin(9600);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);

  // State Setup
  counter = 0;
  pinMode(MASTER_SLAVE_PIN,INPUT);
  masterSlave = digitalRead(MASTER_SLAVE_PIN) == HIGH;

  masterSlave = MASTER;

  // Real version
  if(masterSlave == MASTER){
    state = GO_UNTIL_COLLIDE;
  }else{
    startedSlave = true;
  }

  // Hard coded
  previousState = HALT;

  // motor setup
  pinMode(MTRLFWD,OUTPUT);
  pinMode(MTRRFWD,OUTPUT);
  pinMode(MTRLREV,OUTPUT);
  pinMode(MTRRREV,OUTPUT);
  pinMode(MTRLEN,OUTPUT);
  pinMode(MTRREN,OUTPUT);
  digitalWrite(MTRLEN, LOW);
  digitalWrite(MTRREN, LOW);

  // Color Sensor Setup
  pinMode(REDLED,OUTPUT);
  pinMode(BLUELED,OUTPUT);
  digitalWrite(REDLED, HIGH);
  digitalWrite(BLUELED,LOW);//NOTE: LOW
  colorState = REDSTATE;
  lastColorMeasurement = DARK;
  currentColorMeasurement = DARK;

  // Interrupt Setup for color
  Timer1.initialize(25000); //10 milliseconds
  Timer1.attachInterrupt(colorMeasure);

  // Collision setup
  stopTheMadness = 0;
  debounceTimer = 0;
  collisionTimer = 0;
  collisionTimeout = 0;
  collide = false;
  attachInterrupt(0, collision, RISING); // Interrupt 0 pin 2
}

// ============================== Main Loop ===================================

// While not in the halting state, update the state with the sensors
// then move according to state
void loop() { 
  if(masterSlave == SLAVE){
    state = HALT;
    beSlave();
    previousState = HALT;
    state = GO_UNTIL_COLLIDE;
    masterSlave = MASTER;
  }

  // Prevent collision hardware issues
  if(collide){
    stopTheMadness++;
  }
  if(stopTheMadness > THIS_IS_MADNESS){
    endCollide();
    stopTheMadness = 0;
  }

  // If colliding, reposition
  if(collide == true && state != HALT && state != OVER){
    reposition();
  } 
  
  // If not colliding, move according to state
  if(collide == false){
 //   color = colorValue();
    checkstate();
    move();
  }

  // End program after max runtime
  // if (millis() > MAX_RUNTIME){
  //   state = HALT;
  // }

  delay(40); // DO NOT ERASE

  // For debug purposes
  color = colorValue();
  Serial.print("current color: ");Serial.println(color);
   //Serial.print("current state: ");Serial.println(state);
   //Serial.println("color: ");Serial.println(currentColorMeasurement);
   //Serial.print("Color Value: ");Serial.println(color);

}

//============================= Color Sensor ==================================

// Return current color value
int colorValue() {
  //Serial.println(analogRead(LIGHTSENSOR));
 // Serial.println(colorState);

  if (lastColorMeasurement != currentColorMeasurement) {
    return lastColorReturned;
  }
  
  if (currentColorMeasurement == RED) {
    //Serial.println(RED);
    return (lastColorReturned = RED);
  } else if (currentColorMeasurement == BLUE) {
    //Serial.println(BLUE);
    return (lastColorReturned = BLUE);
  } else {
    //Serial.println(DARK);
    return (lastColorReturned = DARK);
  }
}

// Color measurement ISR
void colorMeasure() {
  if (colorState == BLUESTATE) {
    lastColorMeasurement = currentColorMeasurement;
    blueValue = analogRead(LIGHTSENSOR);
    if(blueValue > BLUE_THRESHOLD) {
      currentColorMeasurement = BLUE;
    }else if (redValue > RED_THRESHOLD) {
      currentColorMeasurement = RED;
    } else {
      currentColorMeasurement = DARK;
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
    // Move forward until you hit a wall
    case GO_UNTIL_COLLIDE:
      //only transitions out via interrupt
      break;

    // Just hit the wall, now looking for RED or BLUE
    case HUNTING_FOR_COLOR:
      if(colorValue() == RED){
        counter = 0;
        state = FOUND_RED_FIRST;
      } else if(colorValue() == BLUE){
        counter = 0;
        state = FOUND_BLUE_FIRST;
      } else{
        state = HUNTING_FOR_COLOR;
      }
      break;

    // Communicate that RED was found, then start tracking RED
    case FOUND_RED_FIRST:
      colorFound(RED_COLOR);
      state = TURN_RED_RIGHT;
      break;
    
    // Communicate that BLUE was found, then start tracking BLUE
    case FOUND_BLUE_FIRST:
      colorFound(BLUE_COLOR);
      state = TURN_BLUE_LEFT;
      break;

    case TURN_BLUE_LEFT:
      if(colorValue() == BLUE){
        state = FOUND_BLUE;
      }
      break;

    case TURN_RED_RIGHT:
      if(colorValue() == DARK){
        state = FOUND_RED;
      }
      break;

    // Seeing RED
    case FOUND_RED:
      if(colorValue() == RED){
        state = FOUND_RED;
      }else{
        counter = 0;
        state = LLOST_RED;
      }
      break;

    // Seeing BLUE
    case FOUND_BLUE:
      if(colorValue() == BLUE){
        state = FOUND_BLUE;
      } else {
        counter = 0;
        state = LLOST_BLUE;
      }
      break;

    // Just lost BLUE, turning left 
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
        state = DONESTATE;
      }
      break;

      // Just lost RED, turning left
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
      } else if(counter < COUNTER_LENGTH * 2 + 2){
        counter ++;
        state = RLOST_RED;
      }else{
        state = DONESTATE;
      }
      break;
    
    // Just hit the wall at the end of the BLUE track, turning around
    case TURN_AROUND_BLUE:
      if(colorValue() == BLUE){
        state = FOUND_BLUE;
      }
      if(counter > rotateTime){
        state = FOUND_BLUE;
      }
      counter++;
      break;
    
    // Just hit the wall at the end of the RED track, turning around
    case TURN_AROUND_RED:
      if(colorValue() == RED){
        state = FOUND_RED;
      }
      if(counter > rotateTime){
        state = FOUND_RED;
      }
      counter++;
      break;

    // Both bots are now done with the course
    case DONESTATE:
      if(startedSlave){
        state = OVER;
        counter = 0;
      }
      else{
        masterSlave = SLAVE;
        finishedMaster();
        beSlave();
        state = OVER;
        counter = 0;
      }
      break;

    case OVER:
      if(counter > 10){
        song();
        state = HALT;
      }else{
        setMotors(REVERSE, SLOW);
        counter++;
      }

      //DO FUN DONE SHIT
    break;

    // The bot is not moving
    default:
      state = HALT;
      break;
  }
}

// Moves the swarmbot according to the state
void move(){
 // Serial.print("move, state: ");Serial.println(state);
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
      setMotors(ROTATER, SLOW);
      break;
    case RLOST_BLUE:
      setMotors(ROTATEL,SLOW);
      break;
    case LLOST_RED:
      setMotors(ROTATEL, SLOW);
      break;
    case RLOST_RED:
      setMotors(ROTATER,SLOW);
      break;
    case TURN_AROUND_BLUE:
      setMotors(ROTATEL, FAST);
      break;
    case TURN_AROUND_RED:
      setMotors(ROTATEL, FAST);
      break;
    case TURN_RED_RIGHT:
      setMotors(ROTATEL, SLOW);
      break;
    case TURN_BLUE_LEFT:
      setMotors(ROTATER, SLOW);
      break;
    // covers halt, wait, and communications states
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
      analogWrite(MTRRFWD,v + MOTOR_OFFSET);
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
    default: // includes "STOPPED" case
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
  volatile unsigned long now = millis();
  if (now < (debounceTimer + BUMPER_DEBOUNCE_MILLIS)){
    return;
  }
  debounceTimer = now;

  // Store previous state if not already colliding
  if(!collide){
    previousState = state;    
  }
  // Set the timeout of the collision movement to 500ms from now
  collisionTimeout = 500;
  // Set the current time of collision
  collisionTimer = millis();
  // Set current state and know that you collided
  state = checkPads();
  // Make sure a pad was actually hit
  collide = true;

  //communicateCollision(); // <-will add this in later once collision is figured out
  
}
 
// return the first pad that got hit, starting the check at PAD1
// Eventually will have different timeouts here
int checkPads(){
  for (int i = PAD0; i < PAD5; i++){
    if(digitalRead(i) == HIGH){
      return i;
    }
  }
  // default to front pad
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

  switch(state){  
    case PAD0: 
      setMotors(REVERSE,MEDIUM);
      if(timeOut()){
        collisionTimeout = 1250;
        if(previousState == GO_UNTIL_COLLIDE){
          collisionTimeout = 1250;
        }
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

    case PAD_HIT:
      setMotors(STOPPED, 0);
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


//============================ Communication ==================================

void colorFound(int foundColor){
  for (; ; ) {
    byte msg;
    Serial.println("In colorFound");
    Serial.print("Sending message: ");
    mask();
    switch(foundColor){
      case RED_COLOR:
        Serial3.write(myRED_FOUND);
        Serial.print("Red Found");
        Serial.println();
        break;
      case BLUE_COLOR:   
        Serial3.write(myBLUE_FOUND);
        Serial.print("Blue Found");
        Serial.println();
        break;
    }

    Serial3.flush();
    unmask();

    delay(PACKET_MILLIS);
    msg = NO_MESSAGE;
    while((Serial3.available()>0)&&(!isValid(msg = Serial3.read())));
    Serial.println();
    if (isValid(msg)==true) {
      Serial.print("Received msg: ");
      Serial.print(msg,DEC);
      Serial.println();
    } else {
      Serial.println("No data heard, timeout");
    }
    if(msg==RECEIVED){
      return;
    }
  }
  lightLED(foundColor);
}
// for tests
void beMaster()
{
  colorFound(RED_COLOR);
  lightLED(RED_COLOR);

}

 void finishedMaster(){
  for (; ; ) {
    mask();
    byte msg;
    Serial.println("In finishedMaster");
    Serial3.write(myDONE_MSG);
       
    Serial3.flush();
    unmask();
    delay(PACKET_MILLIS);
    msg = NO_MESSAGE;
    while((Serial3.available()>0)&&(!isValid(msg = Serial3.read())));
    if (msg != NO_MESSAGE) {
      Serial.print("Received msg");
      Serial.print(msg,DEC);
      Serial.println();
    } else {
      Serial.println("No data heard, timeout");
    }
    if(msg == RECEIVED){
      return;
    }
  }
} 

void beSlave() {
  Serial.println("In slave mode");
  boolean valid = false;
  byte m;
  
  while(!valid){
    m = receiveMessage();
    
    if (isValid(m)){
      valid = true;  
    }
  }

  Serial.print("Received message:");
  Serial.println(m);

  for (int i = 0; i < 50; i++) {
    pingBack();
  }

  if(m == BLUE_FOUND){
    lightLED(BLUE_COLOR); 
  }
  else if(m = RED_FOUND){
    lightLED(RED_COLOR);
  }
  valid = false;
  while(!valid){
    m = receiveMessage();
    
    if (isValid(m)){
      valid = true;  
      delay(5000);//wait for them to back up
    }
  }
  return;
}

void pingBack() {
  mask();
  Serial3.write(myRECEIVED);
  Serial3.flush();
  unmask();
}

void lightLED(int foundColor) {
  Serial.println("Lighting LED");
  if(foundColor == RED_COLOR)
    analogWrite(RED_PIN, 255);
  else
    analogWrite(BLUE_PIN, 255);
}

byte receiveMessage() {
  if (Serial3.available() > 0) {
    byte msg = Serial3.read();
    return msg;
  }
  return 0;  
}

boolean isValid(byte msg) {
  if (msg != 0) Serial.println(msg);
  return ((msg == BLUE_FOUND) || (msg == RED_FOUND) || (msg == DONE_MSG) || (msg == RECEIVED));
}

void mask()
{
  digitalWrite(maskPin, LOW);

}

void unmask()
{
  digitalWrite(maskPin, HIGH);
}


//---------------------------------- Music ------------------------------------
//code for working out the rate at which each note plays and the frequency.
void beep (unsigned char speakerPin, int frequencyInHertz, long timeInMilliseconds){
  int x;      
  long delayAmount = (long)(1000000/frequencyInHertz);
  long loopTime = (long)((timeInMilliseconds*1000)/(delayAmount*2));
  for (x=0;x<loopTime;x++)    
  {    
    digitalWrite(speakerPin,HIGH);
    delayMicroseconds(delayAmount);
    digitalWrite(speakerPin,LOW);
    delayMicroseconds(delayAmount);
  }    
  delay(20);
}   

// Victory
void song() {
  analogWrite(RED_PIN, 0);
  analogWrite(BLUE_PIN, 255);
  beep(SPEAKER_PIN, NOTE_F6, 250);
  analogWrite(BLUE_PIN, 0);
  analogWrite(RED_PIN, 255);
  beep(SPEAKER_PIN, NOTE_G6, 250);
  analogWrite(RED_PIN, 0);
  analogWrite(BLUE_PIN, 255);
  beep(SPEAKER_PIN, NOTE_A6, 2000);
  analogWrite(BLUE_PIN, 0);
  analogWrite(RED_PIN, 255);
  beep(SPEAKER_PIN, NOTE_C7, 1500);
  analogWrite(RED_PIN, 0);
  analogWrite(BLUE_PIN, 255);
  beep(SPEAKER_PIN, NOTE_A6, 500);
  analogWrite(BLUE_PIN, 0);
  analogWrite(RED_PIN, 255);
  beep(SPEAKER_PIN, NOTE_G6, 500);
  analogWrite(BLUE_PIN, 255);
  analogWrite(RED_PIN, 0);
  beep(SPEAKER_PIN, NOTE_F6, 250);
  analogWrite(RED_PIN, 255);
  analogWrite(BLUE_PIN, 0);
  beep(SPEAKER_PIN, NOTE_F6, 1000);
  analogWrite(BLUE_PIN, 0);
  analogWrite(RED_PIN, 0);
}