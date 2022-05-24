#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <U8g2lib.h>
#define ONE_WIRE_BUS D3 //Pin to which is attached a temperature sensor
#include "SmartSensor.h"

//DS18B20

#define VAL_MIN -10.0
#define VAL_MAX 85.0
#define UPDATE_FEQUENCY 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

DeviceAddress devAddr;  //An array device temperature sensors

float tempDev; //Saving the last measurement of temperature
float tempDevLast; //Previous temperature measurement
long lastTemp; //The last measurement

uint8_t sensorOk = false;

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0,  U8X8_PIN_NONE);

sdf::SmartSensor sensor(1,&u8g2);

//Setting the temperature sensor
void initSensor()  {
  DS18B20.begin();

  uint8_t numberOfDevices = DS18B20.getDeviceCount();

  lastTemp = millis();
  DS18B20.requestTemperatures();
  DeviceAddress tmpAddr;
  // Loop through each device, print out address
  for (int i = 0; i < numberOfDevices; i++) {

    if (DS18B20.getAddress(tmpAddr, i)) {
      sensorOk = true;
      DS18B20.getAddress(devAddr, i);
      break;
    }
    //Read temperature from DS18b20
    float tempC = DS18B20.getTempC( devAddr);
  }
}
void updateSensorValues() {
  if (sensor.timeToMeasure())  { //Take a measurement at a fixed time (durationTemp = 5000ms, 5s)

    float tempC = DS18B20.getTempC( devAddr ); //Measuring temperature in Celsius
    tempDev = tempC; //Save the measured value

    DS18B20.setWaitForConversion(false); //No waiting for measurement
    DS18B20.requestTemperatures(); //Initiate the temperature measurement


    sensor.setSensorValue(0, tempDev);

    if(tempDev < VAL_MAX && tempDev > VAL_MIN){
      sensor.setSensorOK(true);
    }else{
      sensor.setSensorOK(false);
    }
  }

}



void initSmartSensor() {
  // Init smart Sensor
  sensor.setID((char *)"ds1");
  sensor.setName((char *)"DS18B20");
  sensor.setType((uint8_t)0, (char *)"temperature");  // used for generating topics (/sensor/dht22/ds42/temperature)
  sensor.setUnit((uint8_t)0,(char *)"C");
  sensor.setMeasuringFrequency(UPDATE_FEQUENCY); // Hz
  sensor.begin();
}

//------------------------------------------
void setup() {

//  Serial.begin(115200);

  // Setup SmartSensor
  initSmartSensor();

  //Setup DS18b20 temperature sensor
  initSensor();
}

void loop() {
  updateSensorValues();
  sensor.tick();
  delay(10);
}
