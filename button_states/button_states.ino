#include "button.h"
uint8_t buttonPin = 14; //button A on the Romi
Button buttonA(buttonPin);

void setup(){
  Serial.begin(115200);
  buttonA.Init(true);
}

unsigned long buttonCount = 0;

void loop(){
  if(buttonA.CheckButtonPress()){
    Serial.println(++buttonCount);
  }
}
