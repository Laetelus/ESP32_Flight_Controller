#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include "IMU.h"
#include "PID.h"

struct ESCValues { int fr, fl, br, bl; };

class Motors {
public:
    Motors() {}
    void Initialize_ESCs();
    void allocatePWMTimers();
    void mix_motors(int throttleInput, const PIDOut& pidOutput);
    void write_motors();
    void idle() { esc_1 = esc_2 = esc_3 = esc_4 = 1000; write_motors(); }
    int  computeESCValue(int, int, int, int);
    // Returns ESC values in physical arm order: FR, FL, BR, BL.
    // esc_1=FR(pin25), esc_3=FL(pin26), esc_2=BR(pin32), esc_4=BL(pin33)
    ESCValues getLastESC() const { return {esc_1, esc_3, esc_2, esc_4}; }
private: 
    static constexpr int MIN_PULSE_LENGTH = 1000;
    static constexpr int MAX_PULSE_LENGTH = 2000;
    // Minimum ESC signal guaranteed to spin all motors once armed.
    // Tune upward if any motor still won't start; keep below hover throttle.
    static constexpr int MOTOR_MIN_SPIN    = 1060;

    static constexpr int esc_pin1 = 25; // FR/CCW  (front-right)
    static constexpr int esc_pin2 = 32; // BR/CW   (back-right  — wire was labelled FL, physically BR)
    static constexpr int esc_pin3 = 26; // FL/CW   (front-left  — wire was labelled BR, physically FL)
    static constexpr int esc_pin4 = 33; // BL/CCW  (back-left)

    Servo 
    esc1, 
    esc2,
    esc3, 
    esc4;
    // Member variables
    int esc_1, esc_2, esc_3, esc_4;

};