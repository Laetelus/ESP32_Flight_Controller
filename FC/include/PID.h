#pragma once
#include <EEPROM.h>
#include "IMU.h"


struct PIDgains {
    float p_gain_roll;
    float i_gain_roll;
    float d_gain_roll;

    float p_gain_pitch;
    float i_gain_pitch;
    float d_gain_pitch;

    float p_gain_yaw;
    float i_gain_yaw;
    float d_gain_yaw;
};

struct PIDOut {
    float pitch = 0.0f; 
    float roll = 0.0f; 
    float yaw = 0.0f; 

    PIDOut() = default;
    PIDOut(float pitchIn, float rollIn, float yawIn)
        : pitch(pitchIn), roll(rollIn), yaw(yawIn) {}
}; 

struct PIDSetpoints {
    float roll; 
    float pitch;
    float yaw;
};

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

    void calculate_pid(const ScaledImuData& gyro, bool integrate = true);
    void reset();

    PIDgains getGains() const {return pid;}
    PIDOut getOutput() const {return pid_output;}
    PIDSetpoints getSetpoints() const {return pid_setpoint;}   
    void setGains(const PIDgains& newGains) {pid = newGains;}
    void setSetpoints(const PIDSetpoints& sp) {pid_setpoint = sp;}
    void setMem(const PIDMem& mem) {pid_mem = mem;}
    void setOutput(const PIDOut& output) {pid_output = output;}

private:
    PIDgains pid; 
    PIDOut pid_output;
    PIDSetpoints pid_setpoint;
    PIDMem pid_mem;

    const int max_roll = 400;     // Practical maximum rate for roll in degrees per second
    const int max_pitch = max_roll;  // Practical maximum rate for pitch in degrees per second
    const int max_yaw = 400;      // Practical maximum rate for yaw in degrees per second

};   