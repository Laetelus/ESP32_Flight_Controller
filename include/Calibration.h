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
};

extern Calibration cal;

#endif