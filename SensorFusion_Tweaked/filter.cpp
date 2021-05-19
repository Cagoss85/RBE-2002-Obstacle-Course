#include "filter.h"

bool ComplementaryFilter::calcAngle(float& observedAngle, float& est, float& gyroBias){
  statusReg = compass.readReg(LSM303::STATUS_A);
  if(bitRead(statusReg, 3)){
    compass.readAcc();
    observedAngle = -1 * atan2(compass.a.x - accXOffset, compass.a.z);
    gyro.read();
    est = estimatedAngle;
    estimatedAngle = estimatedAngle + 0.01 * (gyro.g.y * (8.75/1000) *(PI/180) - gyroBias);
    gyroBias = gyroBias + ep * (estimatedAngle - observedAngle);
    filterResult = observedAngle + k*estimatedAngle - observedAngle;
    return true;
  }
  else{
    return false;
  }
}

bool ComplementaryFilter::handleReadings(){
  static float accelSum = 0;
  static int i = 0;
  if(i < 200){
    accelSum += compass.a.x;
  }
  else if(i == 200){
    accXOffset = accelSum/200;
    i = 0;
    accelSum = 0;
    return false;
  }
  i++;
  return true;
}

void ComplementaryFilter::Init(void){ 
  if (!compass.init()){
    // Failed to detect the compass.
    ledRed(1);
    while(1){
      Serial.println(F("Failed to detect the compass."));
      delay(100);
    }
  }
  
  compass.enableDefault();
  compass.writeReg(LSM303::CTRL1, 0x67); //Set accelerometer sampling rate to 100 Hz
    
  if (!gyro.init()){
    // Failed to detect the gyro.
    ledRed(1);
    while(1){
      Serial.println(F("Failed to detect gyro."));
      delay(100);
    }
  }
  
  gyro.enableDefault(); //default gyro full scale range is 245 dps
  gyro.writeReg(L3G::CTRL1, 0xBF); 
  //Default //set full scale output to 500 dps
  
  while(!Serial){} //wait for the Serial Monitor
  uint8_t ctrl1 = gyro.readReg(L3G::CTRL1);
  
  //Serial.print("CTRL1 is: ");
  //Serial.print(ctrl1, HEX);
  //Serial.print('\n');
  //Accel CTRL2 result comes back to 0x00
  //Accel CTRL1 result comes back to 0x57 => reading at 50 Hz => 0101 0111 => .02 seconds
  //800 Hz = 1001
  //New Value of CTRL1 = 1001 0111 = 0x97 => Sampling rate  = 800 Hz
}
