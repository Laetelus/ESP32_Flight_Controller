
#pragma once

#include <Arduino.h>
#include "I2Cdev.h"
#include <Wire.h>


struct RawImuData {
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
};

struct ScaledImuData {
    float gx_dps = 0.0f;
    float gy_dps = 0.0f;
    float gz_dps = 0.0f;
    float ax_g = 0.0f;
    float ay_g = 0.0f;
    float az_g = 0.0f;
};

struct AccelAngleData {
    float Roll = 0.0f;
    float Pitch = 0.0f;
};

struct FilteredAttitude {
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
};

struct ImuOffsets {
    int16_t gyroX = 0;
    int16_t gyroY = 0;
    int16_t gyroZ = 0;
    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
};

class IMU {
private:
    RawImuData raw_;
    ScaledImuData scaled_;
    AccelAngleData accA_;
    FilteredAttitude attitude_;
    ImuOffsets ofst_;
    float temperatureF_ = 0.0f;
public:
    IMU() {}
    void initializeI2CBus();
    void processIMUData();
    void scaleIMU();
    void calcAccelAngle();

    const RawImuData& getRawData() const {return raw_;}
    const ScaledImuData& getScaledData() const {return scaled_;}
    const AccelAngleData& getAccelAngles() const {return accA_;}
    const FilteredAttitude& getAttitude() const {return attitude_;}
    const ImuOffsets getOffsets() const { return ofst_; }
    void setOffsets(const ImuOffsets& offsets) { ofst_ = offsets; }
};

