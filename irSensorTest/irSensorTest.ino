#include "irSensor.h"
irSensor mySensor;
void setup() {
  Serial.begin(115200);
  mySensor.init();

}
int count = 0;
void loop() {
Serial.println(mySensor.irDetected());
Serial.print('\t');
if(mySensor.irDetected()){
  count++;
}
Serial.print(count);
Serial.print('\n');
}
