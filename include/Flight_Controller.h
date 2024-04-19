#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Calibration.h"
#include "PID_Webserver.h"
#include <Wire.h>

static unsigned long loop_timer = esp_timer_get_time();
struct Flight_Controller
{

  static constexpr int EEPROM_SIZE = 32;
  static constexpr int MIN_PULSE_LENGTH = 1000;
  static constexpr int MAX_PULSE_LENGTH = 2000;
  static constexpr int esc_pin1 = 32; // FR/CCW
  static constexpr int esc_pin2 = 33; // FL/CW
  static constexpr int esc_pin3 = 25; // BR/CW
  static constexpr int esc_pin4 = 26; // BL/CCW

  MPU6050 accelgyro;
  Servo esc1, esc2, esc3, esc4;

  // Member variables
  unsigned long lastDebounceTime;
  const unsigned long debounceDelay = 20;

  // PID Parameters
  float pid_p_gain_roll = 0.0f;             // Gain setting for the roll P-controller
  float pid_i_gain_roll = 0.0f;             // Gain setting for the roll I-controller
  float pid_d_gain_roll = 0.0f;             // Gain setting for the roll D-controller
  float pid_p_gain_pitch = pid_p_gain_roll; // Gain setting for the pitch P-controller.
  float pid_i_gain_pitch = pid_i_gain_roll; // Gain setting for the pitch I-controller.
  float pid_d_gain_pitch = pid_d_gain_roll; // Gain setting for the pitch D-controller.
  float pid_p_gain_yaw = 0.0f;              // Gain setting for the pitch P-controller. //
  float pid_i_gain_yaw = 0.0f;              // Gain setting for the pitch I-controller. //
  float pid_d_gain_yaw = 0.0f;              // Gain setting for the pitch D-controller.

  float roll_level_adjust, pitch_level_adjust;
  float pid_error_temp;
  float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
  float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
  float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
  float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
  float gyroXOffset, gyroYOffset, gyroZOffset, accXOffset, accYOffset, accZOffset;
  float temperatureC;
  float ax_mps2, ay_mps2, az_mps2;

  // Maximum output of the PID-controller (+/-)
  int pid_max_roll = 400;
  int pid_max_pitch = pid_max_roll;
  int pid_max_yaw = 400;

  int esc_1, esc_2, esc_3, esc_4;
  int start;

  int16_t acc_x, acc_y, acc_z, acc_total_vector;
  int16_t gyro_pitch, gyro_roll, gyro_yaw;
  int16_t temp;

  bool isDebounceConditionMet;
  bool gyro_angles_set;
  bool auto_level = true; // Auto level on (true) or off (false)

  // Member functions
  void initialize();
  void initializeI2CBus();
  void initializeGyroAndAccel();
  void performCalibration();
  void setupInputPins();
  void readCurrentTemperature();
  void attachInterrupts();
  void attachESCPins();
  void armESCs();
  void allocatePWMTimers();
  void read_Controller();
  void level_flight();
  void motorControls();
  void calculate_pid();
  void mix_motors();
  void calibrateMPU6050();
  int16_t applyDeadzone(int16_t, int16_t);
  void processIMUData();
  void readGyroData();
  void write_motors();
  void startInitializationSequence();
  float calculatePIDSetpoint(int channel, float level_adjust);
  float calculatePIDSetpointForYaw(int channel_3, int channel_4);
  int computeESCValue(int throttle, int pitch, int roll, int yaw);
  float calculate_pid_component(float input, float setpoint, float &i_mem, float &last_d_error, float p_gain, float i_gain, float d_gain, float max_output);
  bool areMotorsOff();
  void print();
  
  void timer()
  {
    // Check the total time taken for this loop
    unsigned long time_taken = esp_timer_get_time() - loop_timer;

    // If the time taken is more than 4000 microseconds, blink led.
    if (time_taken > 4000)
    {
      digitalWrite(2, HIGH);
      delay(100);
      digitalWrite(2, LOW);
      delay(100);
    }

    // Ensure loop runs at 4000us (250Hz) cycle
    while (esp_timer_get_time() - loop_timer < 4000)
      ;
    loop_timer = esp_timer_get_time();
  }
  
};

extern Flight_Controller flightController;


#endif // FLIGHT_CONTROLLER_h
