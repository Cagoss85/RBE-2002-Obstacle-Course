#include "eventTimer.h"

eventTimer myTimer;
uint64_t max = 10500;

void setup(){
  Serial.begin(115200);
  myTimer.set(500);
  
}

void loop(){
  if(myTimer.checkExpired()){
    if(millis() <= max){
    Serial.println(millis());
    }
    else myTimer.cancel();
    }
}
