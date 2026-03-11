#pragma once
#include <EEPROM.h>
#include "IMU.h"


struct PIDgains {

    float p_gain_roll;
    float i_gain_roll;
    float d_gain_roll;

    float p_gain_pitch = p_gain_roll;
    float i_gain_pitch = i_gain_roll;
    float d_gain_pitch = d_gain_roll;

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

    int max_roll = 90;            // Practical maximum rate for roll in degrees per second
    int max_pitch = max_roll; // Practical maximum rate for pitch in degrees per second
    int max_yaw = 90;             // Practical maximum rate for yaw in degrees per second

}; 

class PID {
public:
    PID() {}
    void calculate_pid();
    void Reset_PID(const AccelAngleData& ang);
    // void Reset_PID();
    PIDgains getGains() const {return pid;}
    void setGains(const PIDgains& newGains) {pid = newGains;}
    PIDOut getOutput() const {return pid_output;}   

private:
    PIDgains pid; 
    PIDOut pid_output;

    float pid_error_temp;

    double pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_last_roll_d_error;
    float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_last_pitch_d_error;
    float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_last_yaw_d_error;
};   