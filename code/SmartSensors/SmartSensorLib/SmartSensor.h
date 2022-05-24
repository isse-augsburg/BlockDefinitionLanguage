#ifndef SmartSensor_h
#define SmartSensor_h

#include "Arduino.h"
#include <U8g2lib.h>
#include <ros2arduino.h>
#include "eeporm_data.h"

// ROS2 Arduino
#define XRCEDDS_PORT  Serial
#define PUBLISH_FREQUENCY 2 //hz

#define BUTTONPIN D3

// Calibration state machine
#define S_IDLE 0
#define S_CALIB 1
#define S_CALC 2
#define CALIB_TIMEOUT 5000
#define MAX_CALIB_VALUES 500

#ifndef CALIBRATION
#define CALIBRATION false
#endif


#define CONFIG_OFFSET 1
 // Byte at MAGIC_OFFSET needs to be 0x42 to indicate the config was initialized
#define MAGIC_OFFSET 0
namespace sdf{




    class SmartSensor
    {
      public:
        SmartSensor(uint8_t num_values, U8G2_SSD1306_128X64_NONAME_1_HW_I2C * u8g2);
        void setID(char * id);
        void setName(char * name);
        void setType(uint8_t n, char * typestr);  // Dadurch werden topics generiert  (/sensor/dht22/42/temperature)
        void setUnit(uint8_t n, char * unitstr);
        void setSensorOK(uint8_t ok);
        void setMeasuringFrequency(float frequency);
        void setSensorValue(uint8_t id, float value);

        void setType(int8_t n, char * typestr);
        float getValue(uint8_t n);
        float getValueCalibrated(uint8_t n);
        void getTopicName(char* buffer, int n);
        void getCalibTopicName(char* buffer, int n);
        void begin();
        void tick();
        uint8_t timeToMeasure();
        void update_calib(float calib_value, uint8_t n);


      protected:
        void setupRos();
        void updateLCD();
        void refreshLCD();
        void setupLCD();
    //    void publish(std_msgs::Float32* msg, void* arg);
        char * _id;
        char * _name;
        char ** _typestrs;
        char ** _unitstrs;
        float * _values;
        float * _values_calibrated;
        uint8_t _num_values; // Number of different sensor values
        uint8_t _sensor_ok; // Sensor value ok
        uint8_t _last_btn; // Last button pin state
        uint8_t _pageno = 0; // Page number for display
        uint8_t  _mavid = 0;
        ros2::Node * _node;
        ros2::Publisher<std_msgs::Float32> ** _publisher;
        unsigned long _last_update = millis();
        unsigned long _last_last_update = millis();
        unsigned long _measure_rate_delay;
        // LCD
        U8G2_SSD1306_128X64_NONAME_1_HW_I2C * _u8g2ptr;//u8g2(U8G2_R0,  U8X8_PIN_NONE);
        long lastLCD;
        const uint16_t LCDUpdateInterval = 500; //The frequency of temperature measurement

        // Calibration
        uint8_t _calib_state = S_IDLE;
        long _last_calib_update;
        float _calib_data_temp[MAX_CALIB_VALUES];
        float _calib_data_error[MAX_CALIB_VALUES];
        uint16_t _calib_values = 0;

        // EEPROM
        void saveConfig(configData_t* cfg);
        void readConfig(configData_t & cfg);
        uint8_t eeprom_initialized();
        void initialize_eeprom();

        configData_t _cfg;
    };


}

#endif // SmartSensor_h
