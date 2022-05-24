#include "Arduino.h"
#include "SmartSensor.h"
#include "linreg.h"
#include <U8g2lib.h>
#include <ros2arduino.h>
#include <EEPROM.h>
#include "eeporm_data.h"
namespace sdf{

      void publish(std_msgs::Float32* msg, void* arg) {
          //(void)(arg);
          msg->data = *((float*)arg);

        }


      void calibCallback(std_msgs::Float32* msg, void* arg){
        //  (void)(arg);
          SmartSensor * sensor = ((SmartSensor*)arg);
          sensor->update_calib(msg->data, 0); // TODO multiple sensors

        }
    SmartSensor::SmartSensor(uint8_t num_values, U8G2_SSD1306_128X64_NONAME_1_HW_I2C * u8g2){

      // Allocate RAM for sensor values
      _typestrs = (char **) malloc(num_values*sizeof(char*));
      _publisher = (ros2::Publisher<std_msgs::Float32>**) malloc(num_values * sizeof(ros2::Publisher<std_msgs::Float32>*));
      _unitstrs = (char **)malloc(num_values*sizeof(char*));
      _values = (float*)malloc(num_values*sizeof(float));
      _values_calibrated = (float*)malloc(num_values*sizeof(float));
      _num_values = num_values;
      _u8g2ptr = u8g2;

  }

    void SmartSensor::update_calib(float calib_value, uint8_t n){
        if (_calib_state == S_IDLE){
          _calib_state = S_CALIB;
        }
        if(_calib_values >= MAX_CALIB_VALUES){
          return;
        }
        // Calib daten speichern
        float measured_value =  getValue(n);
        _calib_data_temp[_calib_values] = measured_value;
        _calib_data_error[_calib_values] = measured_value-calib_value;
        _calib_values++;
        _last_calib_update = millis();
      }



    void SmartSensor::getTopicName(char* buffer, int n){
      if (n>_num_values){return;}

      if(CALIBRATION){
        // (/calibration/temperature)
        getCalibTopicName(buffer, n);
      }else{
        // (/sensor/dht22/42/temperature)
        sprintf(buffer, "mav%d/sensor/%s/%s_%s", _mavid, _typestrs[n],_name,_id);
      }

    }


    void SmartSensor::getCalibTopicName(char* buffer, int n){
      if (n>_num_values){return;}
      // (/calibration/temperature)
      sprintf(buffer, "calibration/%s",  _typestrs[n]);
    }


    void SmartSensor::setMeasuringFrequency(float frequency){
      _measure_rate_delay = 1000.0/frequency;
    }


    float SmartSensor::getValue(uint8_t n){
      if (n>_num_values){return 0;}
        return _values[n];

    }
    float SmartSensor::getValueCalibrated(uint8_t n){
      if (n>_num_values){return 0;}
        return _values_calibrated[n];
    }
    uint8_t SmartSensor::timeToMeasure(){
      return (millis()-_last_update)>_measure_rate_delay;
    }


    void SmartSensor::setID(char * id){
        _id = id;
    }


    void SmartSensor::setName(char * name){
        _name = name;
    }


    void SmartSensor::setType(uint8_t n, char * typestr){
        if(n<_num_values){
            _typestrs[n] = typestr;
        }
    }


    void SmartSensor::setUnit(uint8_t n, char * unitstr){
        if(n<_num_values){
            _unitstrs[n] = unitstr;
        }
    }

    // Initializes the sensor
    void SmartSensor::begin(){
        // Init eeprom
        if(!eeprom_initialized()){
          initialize_eeprom();
        }else{
          readConfig(_cfg);
        }

        // Init ROS
        setupRos();

        // Init LCD
        setupLCD();

        // Init Button
        pinMode(BUTTONPIN, INPUT);
        _last_btn = digitalRead(BUTTONPIN);

        // Init calibration
        if(!CALIBRATION){
          char buff[128];
          for(int i = 0; i< _num_values; i++){
            getCalibTopicName(buff, i);
            _node->createSubscriber<std_msgs::Float32>(buff, (ros2::CallbackFunc)calibCallback, this);
          }
        }
    }

    // should be called in loop()
    void SmartSensor::tick(){
        // Publish to ROS
        ros2::spin(_node);

        // Read Button
        uint8_t btnstate = digitalRead(BUTTONPIN);
        if(btnstate == LOW && _last_btn == HIGH){
          // Button pressed, switch page

          _pageno++;
          if(_pageno>= _num_values){
            _pageno=0;
          }
        }
        _last_btn = btnstate;

        // Handle Calib
        if(_calib_state == S_CALIB){
          if(millis() - _last_calib_update > CALIB_TIMEOUT || _calib_values == MAX_CALIB_VALUES ){
            _calib_state = S_CALC;
            // Update LCD
        //    updateLCD();
            float m = 0, t = 0;
            linreg(_calib_values, _calib_data_temp, _calib_data_error, 4, &m, &t);

            // Write values to EEPROM
            _cfg.sensor_calibrated = true;
            _cfg.calib_m = m;
            _cfg.calib_t = t;
            saveConfig(&_cfg);

            _calib_state = S_IDLE;
            _calib_values = 0;
          //  updateLCD();
          }
        }

        // Update LCD
        refreshLCD();

    }


    void SmartSensor::setSensorOK(uint8_t ok){
        if(ok){
            _sensor_ok = true;
        }else{
            _sensor_ok = false;
        }
    }


    void SmartSensor::setSensorValue(uint8_t n, float value){
      //Serial.println("Sensor Value update");
        if(n<_num_values){
            _values[n] = value;
            _values_calibrated[n] = value;
            if(n == 0){
              // Update timestamps
              _last_last_update = _last_update;
              _last_update = millis();

              if(_cfg.sensor_calibrated){
                _values_calibrated[n] = value + value*_cfg.calib_m + _cfg.calib_t;
              }
          }
        }
    }


    void SmartSensor::setupRos() {
      // Setup ROS Communication
      XRCEDDS_PORT.begin(115200);

      while(_mavid == 0){
        _mavid =  XRCEDDS_PORT.parseInt();
      }

      while (!XRCEDDS_PORT);

      ros2::init(&XRCEDDS_PORT);
/*
      static TempPub TempNode;
      _node = &TempNode;*/
      static ros2::Node tmp_node;
      _node = &tmp_node;
      char buff[128];

      for (int i = 0; i < _num_values; i++){
        getTopicName(buff, i);
        //Serial.println(buff);
        _publisher[i] = _node->createPublisher<std_msgs::Float32>(buff); //TODO More for multiple topics
        //Serial.println("Publisher created");
        _node->createWallFreq(PUBLISH_FREQUENCY, (ros2::CallbackFunc)publish, &_values_calibrated[i], _publisher[i]);
      }


    }


    void SmartSensor::setupLCD() {
      _u8g2ptr->begin();
      _u8g2ptr->setFlipMode(0);
      lastLCD = millis();
    }


    void SmartSensor::updateLCD() {

      const uint8_t headerHeight = 10;

      _u8g2ptr->firstPage();

      char hzBuffer[12];
      float hz = 1000.0/( _last_update-_last_last_update);
      sprintf(hzBuffer, "%.1fHz",hz);


      do {
        _u8g2ptr->setFontMode(1);  // Transparent
        _u8g2ptr->setDrawColor(1);

        _u8g2ptr->setFontDirection(0);
        _u8g2ptr->setFont(u8g2_font_t0_11_tr);

        // Sensor OK?
        _u8g2ptr->drawStr(5, headerHeight, "Sensor:");

        // Calibration OK?
        _u8g2ptr->drawStr(70, headerHeight, "Calib:");

        // Draw Update Rate

        _u8g2ptr->drawFrame(90, 15, 38, 15);
        _u8g2ptr->drawStr(92, 15 + 12, hzBuffer);

        // Draw ID
        _u8g2ptr->drawStr(5, 30, _id);

        // Draw Sensor Value
        _u8g2ptr->setFontDirection(0);
        _u8g2ptr->setFont(u8g2_font_inb16_mf);
        char sensorStr[20];
        //sprintf(sensorStr, "%.1f%s",_values_calibrated[_pageno],_unitstrs[_pageno]);
        sprintf(sensorStr, "%.1f%s",_values[_pageno],_unitstrs[_pageno]);
        //sprintf(sensorStr, "%f",_cfg.calib_m);

        _u8g2ptr->drawStr(35, 60, sensorStr);

        char calibchr;
        if(CALIBRATION){
          calibchr = 'R';
        }else if(_calib_state == S_CALIB){
          calibchr = 'C';

        }else if(_calib_state == S_CALC){
          calibchr =  'L';
        }else {
          calibchr = _cfg.sensor_calibrated  ? 'Y' : 'N';
        }
        // Draw Checkmarks or Xs for Status bits
        _u8g2ptr->setFontDirection(0);
        _u8g2ptr->setFont(u8g2_font_unifont_t_symbols);
        _u8g2ptr->drawGlyph(50, headerHeight, _sensor_ok ? 0x2713 : 'x');
        _u8g2ptr->drawGlyph(110, headerHeight, calibchr);


        _u8g2ptr->drawHLine(0, 15, 128);
      } while (_u8g2ptr->nextPage() );
    }


    void SmartSensor::refreshLCD(){
        long now = millis();
      if ( now - lastLCD > LCDUpdateInterval ) {
        updateLCD();
      }
    }

// EEPROM
void SmartSensor::saveConfig(configData_t* cfg) {
  EEPROM.begin(4095);
  EEPROM.put( CONFIG_OFFSET, *cfg );
  delay(200);
  EEPROM.commit();
  EEPROM.end();
}

uint8_t SmartSensor::eeprom_initialized() {
  EEPROM.begin(4095);
  uint8_t magic = 0;
  EEPROM.get( MAGIC_OFFSET, magic ); // TODO Crazy call by reference thing
  EEPROM.end();

  return magic == 0x42;
}

void SmartSensor::initialize_eeprom() {
  EEPROM.begin(4095);
  uint8_t magic = 0x42;
  EEPROM.put( MAGIC_OFFSET,  magic);
  delay(200);
  EEPROM.commit();
  EEPROM.end();

  EEPROM.begin(4095);
  _cfg.sensor_calibrated = false;
  _cfg.calib_m = 0;
  _cfg.calib_t = 0;
  EEPROM.put( CONFIG_OFFSET,  _cfg);
  delay(200);
  EEPROM.commit();
  EEPROM.end();
}



void SmartSensor::readConfig(configData_t & cfg) {

    EEPROM.begin(4095);
    EEPROM.get( CONFIG_OFFSET, cfg ); // TODO Crazy call by reference thing
    // Ugly but arduino wants this crazy notation

    EEPROM.end();

}


}
