#include "irSensor.h"
#include <Zumo32U4.h>


void irSensor::init(){
    // Enable pull-up resistors on all the sensor inputs.
    FastGPIO::Pin<SENSOR_LEFT>::setInputPulledUp();
    FastGPIO::Pin<SENSOR_RIGHT>::setInputPulledUp();
    FastGPIO::Pin<SENSOR_FRONT>::setInputPulledUp();
}

bool irSensor::irDetected(){
  if(!FastGPIO::Pin<SENSOR_LEFT>::isInputHigh() || !FastGPIO::Pin<SENSOR_RIGHT>::isInputHigh() || !FastGPIO::Pin<SENSOR_FRONT>::isInputHigh()){
    return true;
  }
  else{
    return false;
  }
}
