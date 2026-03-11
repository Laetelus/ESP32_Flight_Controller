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

struct ControlInput {
  int roll;
  int pitch;
  int throttle;
  int yaw;
};

class FC {
public:
  FC(IMU& imuRef, PID& pidRef, Motors& motorsRef) 
        : imu(imuRef), pid(pidRef), motors(motorsRef) {}
  // Member functions
  void initialize();
  void computeControlSetpoints(int &Roll, int &Pitch, int &Throttle, int &Yaw);
  ControlInput updateState();
  void run(); 
  MotorState Motorstate() const {return state;}
  void print();

  // Member variables
  unsigned long lastDebounceTime;
  const unsigned long debounceDelay = 20;

  // int start;

  // float acc_x, acc_y, acc_z, acc_total_vector;
  
  double gyro_pitch, gyro_roll, gyro_yaw;

  bool isDebounceConditionMet;
  bool gyro_angles_set;
  bool auto_level = true; // Auto level on (true) or off (false)

  private: 
    MotorState state = OFF;
    IMU& imu; 
    Motors& motors; 
    PID& pid;
};

