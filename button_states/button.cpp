#include "button.h"

Button::Button(uint8_t pin, uint32_t db){
  buttonPin = pin;
  debouncePeriod = db; 
}

void Button::Init(bool usePullup){
  if(usePullup){
    pinMode(buttonPin, INPUT_PULLUP);
  }
  else{
    pinMode(buttonPin, INPUT);
  }
}

bool Button::CheckButtonPress(void){
  currButtonPos = digitalRead(buttonPin);
  switch(state){
    case BUTTON_STABLE:
      if(currButtonPos != lastButtonPos){
        lastBounceTime = millis();  
        state = BUTTON_UNSTABLE; 
      }
      return false;
      break;
    case BUTTON_UNSTABLE:                                            
      if((millis() - lastBounceTime) >= debouncePeriod){                
        if(currButtonPos == HIGH && lastButtonPos == LOW){          
          lastButtonPos = currButtonPos;                                
          state = BUTTON_STABLE;                                         
          return true;
        } 
        else{
          lastButtonPos = currButtonPos;
          state = BUTTON_STABLE;
        }
        return false;
      }
      break;
  } 
}
