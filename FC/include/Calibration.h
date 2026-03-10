#pragma once
#include <EEPROM.h>
#include "IMU.h"

//Comment to clear EEPROM , uncomment to save calibration values.. 
// #define USE_EEPROM

// class FC; 
class Calibration
{
public:
    static constexpr int EEPROM_SIZE = 32;

    Calibration(IMU& imuRef) : imu(imuRef) {}

    void saveCalibrationValues();
    bool loadCalibrationValues();
    void printStoredCalibrationValues();
    void clearCalibrationData();
    void performCalibration();

private:
    IMU& imu;
};