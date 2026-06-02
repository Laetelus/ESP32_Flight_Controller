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
#include "ReceiverInput.h"

enum MotorState {
    OFF, 
    START, 
    RUNNING
};

// PrintMode controls what FC::print() outputs each cycle.
// Switch by changing the printMode_ member below.
enum PrintMode {
    PRINT_IMU,     // Accel angles vs comp-filter angles — verify IMU fusion
    PRINT_CONTROL, // Sticks → setpoints → gyro rates → PID output — verify control pipeline
    PRINT_MOTORS,  // ESC µs values + state — verify motor mixing
    PRINT_CSV      // Full CSV — all data for Python/Excel capture
};

class FC {
public:
    FC(IMU& imuRef, PID& pidRef, Motors& motorsRef) 
            : imu(imuRef), pid(pidRef), motors(motorsRef) {}
    // Member functions
    void initialize_FC();
    void compute_control_setpoints(const int Roll, const int Pitch, const int Throttle, const int Yaw);
    void update_state(const int throttle, const int yaw);
    void run();
    MotorState motor_state() const {return state;}
    void print();

private: 
    MotorState state = OFF;
    IMU& imu; 
    Motors& motors; 
    PID& pid;
    
    unsigned long lastDebounceTime;
    const unsigned long debounceDelay = 20;
    bool isDebounceConditionMet;
    // When true: angle outer loop subtracts a tilt-proportional rate correction
    // from the stick setpoints, causing the drone to self-level on stick release.
    bool auto_level = true;

    // Software yaw trim (µs). Add this to the raw yaw stick reading before
    // applying the dead zone. Positive values shift the effective centre higher.
    // Set to compensate for a TX yaw stick that rests below the 1492 dead-zone
    // boundary when held normally. Tune by logging yaw_stick at idle and
    // adjusting until sp_yaw stays 0 throughout the RUNNING state.
    static constexpr int YAW_CENTER_TRIM = 15;
    uint32_t printCounter_ = 0; // rate-limits serial output in print()
    PrintMode printMode_   = PRINT_CSV; // change this to switch debug view
    ReceiverPulseSnapshot lastInput_ = {}; // last receiver snapshot, used by print()

};

