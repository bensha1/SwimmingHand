#include <MPU9250_asukiaaa.h>
 
#define SDA_PIN 21
#define SCL_PIN 22
 
MPU9250_asukiaaa mySensor;
 
void setup() {
  while(!Serial);
   
  Serial.begin(9600);
  Serial.println("started");
   
  // for esp32
  Wire.begin(SDA_PIN, SCL_PIN); //sda, scl
   
  mySensor.setWire(&Wire);
   
  mySensor.beginAccel();
  mySensor.beginMag();
   
  // you can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
}
 
void loop() {
  mySensor.accelUpdate();
  Serial.println("print accel values");
  Serial.println("accelX: " + String(mySensor.accelX()));
  Serial.println("accelY: " + String(mySensor.accelY()));
  Serial.println("accelZ: " + String(mySensor.accelZ()));
  Serial.println("accelSqrt: " + String(mySensor.accelSqrt()));
   
  mySensor.magUpdate();
  Serial.println("print mag values");
  Serial.println("magX: " + String(mySensor.magX()));
  Serial.println("maxY: " + String(mySensor.magY()));
  Serial.println("magZ: " + String(mySensor.magZ()));
  Serial.println("horizontal direction: " + String(mySensor.magHorizDirection()));
   
  Serial.println("at " + String(millis()) + "ms");
  delay(500);
}
