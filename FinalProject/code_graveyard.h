void turnVals() {
  myFilter.gyro.read();
  currentYawRate = (myFilter.gyro.g.z * 8.75 / 1000) - 3;
  if(abs(currentYawRate)< 2){
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

void turn(int deg){
  Serial.print("turning");
  if(deg > totalRot){
    motors.setSpeeds(-200,200);
  }
  else if(deg < totalRot){
    motors.setSpeeds(200,-200);
  }
  else{
    motors.setSpeeds(0,0);
  }
}
 * //void handlePlatform(){
//  if(state == DEAD){
//    offRamp = false;
//    Serial.println("Handling Platform");
//    //Serial.print('\t');
//    //targetLeft = 0;
//    //targetRight = 0;
//    //myTimer.start(1000); //start a timer to calm the robot down  
//  }
//
//}


//Function works as a bool but outputs true for a second, interferred with the event driven programming
static float prevPitch = 0;
static float currPitch = 0;
bool pitchChange(){
  if(myFilter.calcAngle(myFilter.observedAngle, myFilter.estimatedAngle, myFilter.gyroBias)){
    myTimer.start(500);
    if(myTimer.checkExpired()){
      prevPitch = currPitch;
      currPitch = myFilter.filterResult * 180/PI;
    }
  }
  else if(abs(currPitch - prevPitch) > 5){
    return true;
  }
  else{
    return false;
  }
}

//event driven handler for handling the pitch
void handlePitch(){ //the robots pitch changed
  Serial.print("handling pitch");
  if(i == 1){ //since i = 1. This must mean we are on the ramp
    i++;  //change it to 2. But keep going
  }
  if(i == 2){ //the pitch changed again. The robot must've flattened out
    motors.setSpeeds(400,400); //stop driving
    Serial.println("motors");
    myTimer.start(5000); //start a timer to calm the robot down
  }
}
*/
