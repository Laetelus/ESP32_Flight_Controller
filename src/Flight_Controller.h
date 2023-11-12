//FLIGHT_CONTROLLER_h
#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H


#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>
#include "Flight_Controller.h"

class Flight_Controller {
public:

    void initialize(); // Initialize the quadcopter
    void read_Controller();
    void level_flight();
    void motorControls();
    void calculate_pid();
    void mix_motors();
    void calibrateMPU6050(); 
    int16_t applyDeadzone(int16_t,int16_t);
    void processIMUData();
    void saveCalibrationValues();
    bool loadCalibrationValues(); 
    void printStoredCalibrationValues(); 
    void clearCalibrationData();
    void readGyroData();
    void write_motors(); 
    void print_gyro_data();

private:
    
    //Allocate 32 bytes 
    #define EEPROM_SIZE 32

    // Constants
    static constexpr int MIN_PULSE_LENGTH = 1000;
    static constexpr int MAX_PULSE_LENGTH = 2000;

    // Controller
    static constexpr int THROTTLE = 36; 
    static constexpr int YAW = 39; // rudder
    static constexpr int PITCH = 34; // elevator
    static constexpr int ROLL = 35; // aileron

    // ESC Pins
    static constexpr int esc_pin1 = 32; // FR/CCW
    static constexpr int esc_pin2 = 33; // FL/CW
    static constexpr int esc_pin3 = 25; // BR/CW
    static constexpr int esc_pin4 = 26; // BL/CCW

    // PID Parameters
    float pid_p_gain_roll = 1.3;
    float pid_i_gain_roll = 0.04;
    float pid_d_gain_roll = 18.0;
    int pid_max_roll = 400;

    float pid_p_gain_pitch = pid_p_gain_roll;
    float pid_i_gain_pitch = pid_i_gain_roll;
    float pid_d_gain_pitch = pid_d_gain_roll;
    int pid_max_pitch = pid_max_roll;

    float pid_p_gain_yaw = 4.0;
    float pid_i_gain_yaw = 0.02;
    float pid_d_gain_yaw = 0.0;
    int pid_max_yaw = 400;

    // Other variables
    int esc_1, esc_2, esc_3, esc_4;
    int throttle;
    int start;
    float roll_level_adjust, pitch_level_adjust;

    int16_t acc_x, acc_y, acc_z, acc_total_vector;
    int16_t gyro_pitch, gyro_roll, gyro_yaw;
    float pid_error_temp;
    float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
    float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
    float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
    float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
    float gyroXOffset = 0.0, gyroYOffset = 0.0, gyroZOffset = 0.0, accXOffset = 0.0, accYOffset = 0.0, accZOffset = 0.0;
    int16_t temp = 0; 
    float temperatureC = 0.0f;   
    boolean gyro_angles_set;
    boolean auto_level = true; //Auto level on (true) or off (false)
   
    // initialize  units of g
    float ax_mps2 = 0.0f; 
    float ay_mps2 = 0.0f; 
    float az_mps2 = 0.0f; 

    // Receiver input variables
    volatile int16_t receiver_input_channel_3;
    volatile int16_t receiver_input_channel_4;
    volatile int16_t receiver_input_channel_1;
    volatile int16_t receiver_input_channel_2;

    // MPU6050 instance
    MPU6050 accelgyro;

    // Servo motors
    Servo esc1, esc2, esc3, esc4;
};

#endif // FLIGHT_CONTROLLER_h