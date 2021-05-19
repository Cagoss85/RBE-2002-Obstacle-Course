#include <Arduino.h>
uint8_t irPin = A6; //pin used to connect the sensor
volatile uint32_t lastTime = 0;
uint32_t interval = 100;
float volt = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial){}
  Serial.println("Hi!");
  pinMode(irPin,INPUT);
}
int count = 0;
void loop() {
  if(millis() - lastTime >= interval){
    //Serial.print(millis());
    //Serial.print('\t');
    Serial.print("ADC = ");
    Serial.print(sharp_ir());
    Serial.print('\t');
    Serial.print("Voltage = ");
    Serial.print(voltage());
    Serial.print('\t');
    Serial.print("Distance = ");
    Serial.print(distance());    
    Serial.print('\n');
    lastTime = millis();   
  }
}

//input range 10-80cm
//5V ref voltage
//10-bit
//cast ADC output as float

bool irDetected(){
  if(analogRead(irPin) < 0){
    return true;
  }
}

int sharp_ir(void){
  uint16_t val = 0;
  val = analogRead(irPin);
  return val;
}

float distance(void){
  float dist = 0;
  dist = (19.997/(voltage() - .0911));
  return dist;
}

float voltage(void){
  volt = ((float)sharp_ir()/1024) * 5;
  return volt;
}
