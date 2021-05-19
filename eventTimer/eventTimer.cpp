#include "eventTimer.h"

void eventTimer::set(uint32_t interval){
  duration = interval;
  isRunning = true;
}

bool eventTimer::checkExpired(void){
  if(((millis()- lastTime) >= (duration)) && isRunning == true){
    lastTime = millis();
    return true;
  }
  else return false;
}

void eventTimer::cancel(void){
  isRunning = false;
}
