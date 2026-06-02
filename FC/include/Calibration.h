#pragma once
#include <EEPROM.h>
#include "IMU.h"

//Un comment to clear EEPROM , comment to save calibration values.. 
#define CLEAR_EEPROM

class Calibration
{
public:
    static constexpr int EEPROM_SIZE = 32;
    
    Calibration(IMU& imuRef) : imu(imuRef) {}
    void saveCalibrationValues();
    bool loadCalibrationValues();
    void printStoredCalibrationValues();
    void clearCalibrationData();
    void CalibrateIMU();

private:
    IMU& imu;
};