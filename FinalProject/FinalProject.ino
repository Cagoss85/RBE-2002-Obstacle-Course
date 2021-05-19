#include "functions.h"    //contains all functions for state machine
#include <Zumo32U4Motors.h>     //zumo motors
#include <Zumo32U4Encoders.h>   //zumo encoders
#include <Wire.h>               //arduino wire library
#include <Zumo32U4LCD.h>

#include <button.h> 
#include <eventTimer.h>    //timer functions
#include <irSensor.h>      //IR functions
#include <filter.h>        //IMU comp filter

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LCD lcd;
Button buttonA(14);
eventTimer myTimer;
LineSensor myLineSensors;
irSensor myIR;
ComplementaryFilter myFilter;

enum ROBOT_STATES{IDLE, WAIT, WALL, TURN, LINE, DEAD, SPIN};
ROBOT_STATES state = IDLE;

uint8_t n = 0;
int i = 0;

bool readyToPID = false;
bool readyToWallFollow = false;
bool readyToLineFollow = false;
bool readyToTurn = false;
bool readyToDeadReckon = false;
bool onRamp = false;
bool offRamp = false;
bool readyToSpin = false;
bool getHeading = false;

float dist = 0;
float volt = 0;

volatile int16_t countsLeft = 0;
volatile int16_t countsRight = 0;

float effortLeft = 0, effortRight = 0;

float amtTurned = 0;
float currentYawRate = 0;
float seconds = 0;
float prevGyro = 0;
float totalRot = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  buttonA.Init();
  myLineSensors.initLineSensors();
  myIR.init(); 
  myFilter.Init();
  lcd.init();

  noInterrupts();
  //sets up timer 4
  TCCR4A = 0x00; //disable some functionality -- no need to worry about this
  TCCR4B = 0x0C; //sets the prescaler -- look in the handout for values
  TCCR4C = 0x04; //toggles pin 6 at one-half the timer frequency
  TCCR4D = 0x00; //normal mode
 
  OCR4C = 0x83;   //TOP goes in OCR4C
  TIMSK4 = 0x04; //enable overflow interrupt
  interrupts(); //re-enable
}

void loop() {
  if(buttonA.CheckButtonPress())    handleButtonPress();
  if(myTimer.checkExpired())        handleTimer();
  if(readyToWallFollow)             wallFollow();
  if(readyToPID)                    handlePID();
  if(myLineSensors.lineDetected())  handleLine();
  if(readyToTurn)                   handleTurn();
  if(readyToLineFollow)             lineFollow();
  if(myIR.irDetected())             handleIR();
  if(readyToDeadReckon)             handleDeadReckon();
  if(myFilter.calcAngle(myFilter.observedAngle, myFilter.estimatedAngle, myFilter.gyroBias)){
    handleIMU();
  }
  if(getHeading)                    turnVals();
  if(readyToSpin)                 handleSpin(230);

  Serial.print(state);
  Serial.print('\t');
  Serial.print(abs(myFilter.filterResult));
  Serial.print('\t');
  Serial.print(dist);
  Serial.print('\t');
  Serial.print(targetLeft);
  Serial.print('\t');
  Serial.print(targetRight);
  Serial.print('\t');  
  Serial.print(effortRight);
  Serial.print('\t');
  Serial.print(totalRot);
  Serial.print('\n');
}
/**___________________________________________________________________________________________________________________________________________________________________________________**/
void handleButtonPress(){
  if(state == IDLE){
    state = WAIT;
    myTimer.start(1000);
  }
}
/*___________________________________________________________________________________________________________________________________________________________________________________*/
void handleTimer(){
  if(state == WAIT){
    n++;  //n = 1
    state = WALL; //go to the wall state
    //Serial.print("GO!");
    //Serial.print('\t');
  }
  else if(state == TURN){ //since the timer expired for a turn
    if(n == 1){ //and n = 1
      //Serial.print("Expired");
      state = LINE; //now we are gonna go the line follow state
      n++; //n = 2
      readyToLineFollow = true; //set flag for line following
    }
    else if(n == 2){ //and n = 2
      state = DEAD; //now we are gonna go to the dead reckon state
      readyToDeadReckon = true; //set the flag for dead reckoning
    } 
  }//end TURN state
  else if(state == DEAD){
    Serial.print("readyTOSPin");
    state = SPIN;
    getHeading = true;
    readyToSpin = true;
  }
}//end function
/*___________________________________________________________________________________________________________________________________________________________________________________*/
void handleLine(){ //you detected a line
  if(state == WALL){  //and are in the wall state
    state = TURN; //we are gonna to the turn state and stop wall following
    readyToTurn = true; //set flag for turning
    //Serial.print("ready to turn");
    //Serial.print('\t');
  }
}
/*___________________________________________________________________________________________________________________________________________________________________________________*/
void handleTurn(){ //since the flag has been set
  if(state == TURN){ //and we are in the turn state now
    if(n == 1){
      targetLeft = -5;
      targetRight = 25;
      myTimer.start(500); //start the timer for it
      readyToTurn = false;   
    }
    else if(n == 2){
      targetLeft = -23;
      targetRight = 25;
      myTimer.start(375);
      readyToTurn = false;
    }
  }
}
/*___________________________________________________________________________________________________________________________________________________________________________________*/
void handleIR(){ //you detected an IR signal
  if(state == LINE){  //and you are currently line following
    //Serial.print("IR Detected");
    //Serial.print('\t');
    state = TURN; //go to the turn state
    readyToTurn = true; //set the flag for turning
  }
}
/*___________________________________________________________________________________________________________________________________________________________________________________*/
void handleDeadReckon(){ //since the flag has been set
  if(state == DEAD && !offRamp && !onRamp){ //and we are in the dead reckoning state
    targetLeft = 15;
    targetRight = 14;
  }
}
/*___________________________________________________________________________________________________________________________________________________________________________________*/
void handleIMU(){
  if(state == DEAD){
    //Serial.println("Handling ");
    if((myFilter.filterResult < -20) && offRamp == false){
      Serial.println("On Ramp");
      //Serial.print('\t');
      onRamp = true;
      offRamp = false;
    }
    else if((myFilter.filterResult >  1) && onRamp == true){
      Serial.println("On Platform");
      //Serial.print('\t');
      onRamp = false;
      offRamp = true;
      targetLeft = 0;
      targetRight = 0;
      myTimer.start(750);
    }
  }
}
/*___________________________________________________________________________________________________________________________________________________________________________________*/

/*___________________________________________________________________________________________________________________________________________________________________________________*/
void handleSpin(int deg){ //since the flag has been set
  turnVals();
  if(state == SPIN){ //and we are in the spin state
    Serial.print("turning");
    if(totalRot < deg){
      motors.setSpeeds(-250, 250);
    }
    else{
      motors.setSpeeds(0,0);
      readyToSpin = false; //set the flag to false to end the spin
      state = IDLE;
    }
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*___________________________________________________________________________________________________________________________________________________________________________________*/
void lineFollow(){
  if(state == LINE){
    myLineSensors.readThem();
    readyToLineFollow = 0;
  
    sensorError = (myLineSensors.lineDetector[0] - myLineSensors.lineDetector[1]); //center is the target
    sumError += sensorError;
  
    float sensorDerivative = sensorError - lastSensorError;
    lastSensorError = sensorError;
  
    float adj = Kp3 * sensorError + Ki3 * sumError + Kd3 * sensorDerivative;

    targetLeft = targetSpeed - adj;
    targetRight = targetSpeed + adj; 
  }
}
/*___________________________________________________________________________________________________________________________________________________________________________________*/
void wallFollow(){
  readyToWallFollow = 0;
    //Serial.println("Wall Following");
    volt = ((float)analogRead(A0) * 5) / 1024;
    dist = 19.997 / (volt - .0911);
    
    static float lastIRError = 0;
    static float irSum = 0;

    float irError = targetDist - dist;
    irSum += irError;
    float irDeriv = irError - lastIRError;
    lastIRError = irError;

    float adj2 = Kp2 * irError + Ki2 * irSum + Kd2 * irDeriv;
    if(state == WALL){
    targetLeft = targetSpeed - adj2;
    targetRight = targetSpeed + adj2;
  }
}
/*___________________________________________________________________________________________________________________________________________________________________________________*/
void handlePID(){
  if(state != SPIN){
    readyToPID = 0; //clear the timer flag
    //for tracking previous counts
    static int16_t prevLeft = 0;
    static int16_t prevRight = 0;

    //error sum
    static int16_t sumLeft = 0;
    static int16_t sumRight = 0;
    //Serial.print ("hi");

    noInterrupts();
    int16_t speedLeft = countsLeft - prevLeft;
    int16_t speedRight = countsRight - prevRight;

    prevLeft = countsLeft;
    prevRight = countsRight;
    interrupts();

    int16_t errorLeft = targetLeft - speedLeft;
    sumLeft += errorLeft;
    if (sumLeft > 400) {
     sumLeft = 400;
    }
    if (sumLeft < -400) {
      sumLeft = -400;
    }
    
    int16_t errorRight = targetRight - speedRight;
    sumRight += errorRight;
    if (sumRight > 400) {
      sumRight = 400;
    }
    if (sumRight < -400) {
      sumRight = -400;
    }

    effortLeft = Kp * errorLeft + Ki * sumLeft ;
    effortRight = Kp * errorRight + Ki * sumRight;

    motors.setSpeeds(effortLeft, effortRight);  
  }
  else{} 
}
/*___________________________________________________________________________________________________________________________________________________________________________________*/
void turnVals() {
  myFilter.gyro.read();
  currentYawRate = (myFilter.gyro.g.z * 8.75 / 1000) - 3;
  if(abs(currentYawRate) < 2){
    currentYawRate = 0;
  }
  Serial.print(currentYawRate);
  Serial.print('\t');
  float milli = millis();
  seconds = (milli - prevGyro);
  Serial.print(seconds);
  Serial.print('\t');
  prevGyro = milli;
  amtTurned = currentYawRate * (seconds / 1000);
  Serial.print(amtTurned);
  Serial.print('\t');
  totalRot += amtTurned;
  Serial.print(totalRot);
  Serial.print('\n');
}
/*
void spin(int deg){
  Serial.print("turning");
  if(deg > totalRot){
    motors.setSpeeds(-200,200);
  }
  else{
    motors.setSpeeds(0,0);
  }
}
*/
/*____________________________________________________________________________________________________________________________________________________________________________________*/
ISR(TIMER4_OVF_vect)
{
  //Capture a "snapshot" of the encoder counts for later processing
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();
  readyToLineFollow = 1;
  readyToWallFollow = 1;
  readyToPID = 1;
}
