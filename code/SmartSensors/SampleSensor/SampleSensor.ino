#include <Arduino.h>
#include <U8g2lib.h>
#include "SmartSensor.h"

#define VAL_MIN -10.0
#define VAL_MAX 85.0

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0,  U8X8_PIN_NONE);
sdf::SmartSensor sensor(1,&u8g2);

void initSensor()  {
  // TODO: Custom Sensor init Code
}

void updateSensorValues() {
  // Cusotm Sensor Code
  if (sensor.timeToMeasure()) {

    float value = // TODO: Custom Sensor code
    sensor.setSensorValue(0, value);

    // Simple plausibility check
    if(tempDev < VAL_MAX && tempDev > VAL_MIN){
      sensor.setSensorOK(true);
    }else{
      sensor.setSensorOK(false);
    }
  }
}

void initSmartSensor() {
  // Init smart Sensor
  sensor.setID((char *)"ds42");
  sensor.setName((char *)"DS18B20");
  sensor.setType((uint8_t)0, (char *)"temperature");  // used for generating topics e.g. (/sensor/dht22/ds42/temperature)
  sensor.setUnit((uint8_t)0,(char *)"C");
  sensor.setMeasuringFrequency(5); // Hz
  sensor.begin();
}

void setup() {

  Serial.begin(115200);
  // Setup SmartSensor
  initSmartSensor();
  // Setup actual sensor
  initSensor();
}

void loop() {
  updateSensorValues();
  sensor.tick();
  delay(10);
}
