#pragma once
#include <EEPROM.h>
#include "IMU.h"


struct PIDgains {
    float p_gain_roll  = 1.3f;
    float i_gain_roll  = 0.04f;
    float d_gain_roll  = 18.0f;

    float p_gain_pitch = 1.3f;
    float i_gain_pitch = 0.04f;
    float d_gain_pitch = 18.0f;

    float p_gain_yaw   = 4.0f;
    float i_gain_yaw   = 0.02f;
    float d_gain_yaw   = 0.0f;
};

struct PIDOut {
    float pitch = 0.0f; 
    float roll = 0.0f; 
    float yaw = 0.0f; 

    PIDOut() = default;
    PIDOut(float pitchIn, float rollIn, float yawIn)
        : pitch(pitchIn), roll(rollIn), yaw(yawIn) {}
}; 
struct PIDLimits {
    int roll = 90;     // Practical maximum rate for roll in degrees per second
    int pitch = roll;  // Practical maximum rate for pitch in degrees per second
    int yaw = 90;      // Practical maximum rate for yaw in degrees per second
}; 

struct PIDSetpoints {
    float roll; 
    float pitch;
    float yaw;
};

// struct for PID mem roll 
struct PIDMem {
    float i_mem_roll;
    float last_roll_d_error;

    float i_mem_pitch;
    float last_pitch_d_error;

    float i_mem_yaw;
    float last_yaw_d_error;
};

class PID {
public:
    PID() {}

    void calculate_pid(const ScaledImuData& gyro);
    void reset();

    PIDgains getGains() const {return pid;}
    PIDOut getOutput() const {return pid_output;}   
    void setGains(const PIDgains& newGains) {pid = newGains;}
    void setSetpoints(const PIDSetpoints& sp) {pid_setpoint = sp;}
    void setMem(const PIDMem& mem) {pid_mem = mem;}
    void setOutput(const PIDOut& output) {pid_output = output;}

private:
    PIDgains pid; 
    PIDOut pid_output;
    PIDLimits pid_max;
    PIDSetpoints pid_setpoint;
    PIDMem pid_mem;

    double gyro_roll_input;
    float gyro_pitch_input;
    float gyro_yaw_input;
};   