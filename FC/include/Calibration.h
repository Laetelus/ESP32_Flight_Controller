#ifndef CALIBRATION
#define CALIBRATION

#include <EEPROM.h>
#include "Flight_Controller.h"

struct Calibration
{
    void saveCalibrationValues();
    bool loadCalibrationValues();
    void printStoredCalibrationValues();
    void clearCalibrationData();
    void performCalibration();

};

extern Calibration cal;

#endif