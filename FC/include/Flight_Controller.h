#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Calibration.h"
#include "PID_Webserver.h"
#include "Kalman.h"
#include <Wire.h>
#include "IMU.h"
#include "Motors.h"
#include "PID.h"

enum MotorState {
    OFF, 
    START, 
    RUNNING
};

class FC {
public:
    FC(IMU& imuRef, PID& pidRef, Motors& motorsRef) 
            : imu(imuRef), pid(pidRef), motors(motorsRef) {}
    // Member functions
    void initialize();
    void computeControlSetpoints(const int Roll, const int Pitch, const int Throttle, const int Yaw);
    void updateState(const int throttle, const int yaw);
    void run(); 
    MotorState Motorstate() const {return state;}
    void print();

private: 
    MotorState state = OFF;
    IMU& imu; 
    Motors& motors; 
    PID& pid;
    
    unsigned long lastDebounceTime;
    const unsigned long debounceDelay = 20;
    bool isDebounceConditionMet;
    // bool auto_level — reserved for future cascade outer loop

};

