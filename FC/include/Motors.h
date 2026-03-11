#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include "IMU.h"
#include "PID.h"

class Motors {
public:
    Motors() {}
    void Initialize_ESCs();
    void allocatePWMTimers();
    void mix_motors(int throttleInput, const PIDOut& pidOutput, int state);
    void write_motors();
    int  computeESCValue(int, int, int, int);
private: 
  static constexpr int MIN_PULSE_LENGTH = 1000;
  static constexpr int MAX_PULSE_LENGTH = 2000;

  // new oriant
  static constexpr int esc_pin1 = 25; // FR/CCW
  static constexpr int esc_pin2 = 32; // FL/CW
  static constexpr int esc_pin3 = 26; // BR/CW
  static constexpr int esc_pin4 = 33; // BL/CCW

  Servo 
  // FR/CCW
  esc1, 
 // FL/CW
  esc2,
  // BR/CW 
  esc3, 
  // BL/CCW
  esc4;
      // Member variables
  int esc_1, esc_2, esc_3, esc_4;

};