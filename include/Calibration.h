#ifndef CALIBRATION
#define CALIBRATION

#include <EEPROM.h> 
#include "Flight_Controller.h"
#include <EEPROM.h>
// struct Flight_Controller;

struct Calibration 
{
  
    void saveCalibrationValues();
    bool loadCalibrationValues();
    void printStoredCalibrationValues();
    void clearCalibrationData();
}; 

// extern Flight_Controller flightController;


#endif 