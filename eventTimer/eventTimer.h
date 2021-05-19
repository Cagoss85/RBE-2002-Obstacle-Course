 #include "Arduino.h"
#ifndef __EVENTTIMER_H
#define __EVENTTIMER_H

class eventTimer{
  private:
    uint32_t duration;
    uint32_t lastTime = 0;
    bool isRunning = false;
  public:
    void set(uint32_t interval = 0);
    bool checkExpired(void);
    void cancel(void);  
};

#endif
    
