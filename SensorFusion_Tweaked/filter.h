#include <Wire.h>
#include <Zumo32U4.h>
#include <Zumo32U4Motors.h>
#ifndef __FILTER_H
#define __FILTER_H

class ComplementaryFilter{
  private:
 
  public:
    float ep = .0075;
    float k = .375;
    float gyroBias = 0;
    float accXOffset = 0;
    float estimatedAngle = 0;
    float observedAngle = 0;
    uint8_t statusReg;
    LSM303 compass;
    L3G gyro;
    void Init(void);
    bool calcAngle(float& observedAngle, float& est, float& gyroBias);
    bool handleReadings();
    float filterResult;
};

#endif
