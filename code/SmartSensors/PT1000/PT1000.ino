#include <Arduino.h>
#include <U8g2lib.h>
#include "SmartSensor.h"
#include <float.h>
#include <Wire.h>

// I2C Address of TEMOD PT1000 Board
#define DEV_ID 0x78


#define TEMP_MIN -10.0
#define TEMP_MAX 85.0

#define UPDATE_FREQUENCY .5


U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0,  U8X8_PIN_NONE);
sdf::SmartSensor sensor(1, &u8g2);

void initSensor()  {
 // Nothing to do here. Wire library is initialized in the oled library
}

void updateSensorValues() {
  int Th, Tl = 0;
  // Cusotm Sensor Code
  if (sensor.timeToMeasure()) {
    uint8_t ok = true;
    float temp = 0.0;

    Wire.requestFrom(DEV_ID, 2); // read 2 bytes
    Th = Wire.read(); // receive 1st byte
    Tl = Wire.read(); // receive 2nd byte
    if (Th == -1 || Tl == -1 ) {
      ok = false;
    } else {
      temp = float(((Th << 8 | Tl)) / 256.0) - 32.0;
    }

    // Simple plausibility check
    if (temp > TEMP_MAX || temp < TEMP_MIN) {
      ok = false;
    }
    sensor.setSensorValue(0, temp);
    sensor.setSensorOK(ok);
  }
}

void initSmartSensor() {
  // Init smart Sensor
  sensor.setID((char *)"pt1");
  sensor.setName((char *)"PT1000");
  sensor.setType((uint8_t)0, (char *)"temperature");  // used for generating topics e.g. (/sensor/dht22/ds42/temperature)
  sensor.setUnit((uint8_t)0, (char *)"C");
  sensor.setMeasuringFrequency(UPDATE_FREQUENCY); // Hz
  #define CALIBRATION true
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
