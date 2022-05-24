#include <Arduino.h>
#include <U8g2lib.h>
#include "SmartSensor.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <float.h>

#define DHTPIN D5     // Digital pin connected to the DHT sensor
#define PWR_PIN D6
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --

#define DHTTYPE    DHT22     // DHT 22 (AM2302)


DHT_Unified dht(DHTPIN, DHTTYPE);

#define TEMP_MIN -10.0
#define TEMP_MAX 85.0
#define HUMIDITY_MIN 0.0
#define HUMIDITY_MAX 100.0

#define UPDATE_FREQUENCY 4


U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0,  U8X8_PIN_NONE);
sdf::SmartSensor sensor(2,&u8g2);

void initSensor()  {
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);

  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);


  // Power Cycle DTH, otherwise data can't be read
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(200);
  digitalWrite(PWR_PIN, HIGH);

}

void updateSensorValues() {
  // Cusotm Sensor Code
  if (sensor.timeToMeasure()) {
    uint8_t ok = true;
    float temp = FLT_MIN, humidity = FLT_MIN;
    sensors_event_t event;
    dht.temperature().getEvent(&event);

    if (isnan(event.temperature)) {
      ok = false;
    }else {
      temp = event.temperature;
    }

    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      ok = false;
    }
    else {
      humidity = event.relative_humidity;
    }
    // Simple plausibility check
    if(temp > TEMP_MAX || temp < TEMP_MIN){
      ok = false;
    }

    if(humidity > HUMIDITY_MAX || humidity < HUMIDITY_MIN){
      ok = false;
    }
    sensor.setSensorValue(0, temp);
    sensor.setSensorValue(1, humidity);
    sensor.setSensorOK(ok);
  }
}

void initSmartSensor() {
  // Init smart Sensor
  sensor.setID((char *)"dht5");
  sensor.setName((char *)"DHT22");
  sensor.setType((uint8_t)0, (char *)"temperature");  // used for generating topics e.g. (/sensor/dht22/ds42/temperature)
  sensor.setUnit((uint8_t)0,(char *)"C");
  sensor.setType((uint8_t)1, (char *)"humidity");  
  sensor.setUnit((uint8_t)1,(char *)"%");
  sensor.setMeasuringFrequency(UPDATE_FREQUENCY); // Hz
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
