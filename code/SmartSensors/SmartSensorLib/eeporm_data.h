#ifndef eeprom_data_h
#define eeprom_data_h

#include <WString.h>

// Types 'byte' und 'word' doesn't work!
typedef struct {
  uint8_t sensor_calibrated;
  float calib_m;
  float calib_t;
} configData_t;

#endif
