#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Calibration.h"
#include "PID_Webserver.h"
#include "Kalman.h"
#include <Wire.h>

struct Flight_Controller
{
#define THROTTLE 36
#define YAW 39
#define ROLL 35
#define PITCH 34

  static constexpr int EEPROM_SIZE = 32;
  static constexpr int MIN_PULSE_LENGTH = 1000;
  static constexpr int MAX_PULSE_LENGTH = 2000;

  // static constexpr int esc_pin1 = 32; // FR/CCW
  // static constexpr int esc_pin2 = 33; // FL/CW
  // static constexpr int esc_pin3 = 25; // BR/CW
  // static constexpr int esc_pin4 = 26; // BL/CCW

  // new oriant
  static constexpr int esc_pin1 = 25; // FR/CCW
  static constexpr int esc_pin2 = 32; // FL/CW
  static constexpr int esc_pin3 = 26; // BR/CW
  static constexpr int esc_pin4 = 33; // BL/CCW

  Kalman kalmanRoll;
  Kalman kalmanPitch;

  Servo esc1, esc2, esc3, esc4;

  // Member variables
  unsigned long lastDebounceTime;
  const unsigned long debounceDelay = 20;

  // PID Parameters
  float pid_p_gain_roll = 0.0f; // Gain setting for the roll P-controller
  float pid_i_gain_roll = 0.0f; // Gain setting for the roll I-controller
  float pid_d_gain_roll = 0.0f; // Gain setting for the roll D-controller

  float pid_p_gain_pitch = pid_p_gain_roll; // Gain setting for the pitch P-controller.
  float pid_i_gain_pitch = pid_i_gain_roll; // Gain setting for the pitch I-controller.
  float pid_d_gain_pitch = pid_d_gain_roll; // Gain setting for the pitch D-controller.

  float pid_p_gain_yaw = 0.0f; // Gain setting for the yawn P-controller.
  float pid_i_gain_yaw = 0.0f; // Gain setting for the yawn I-controller.
  float pid_d_gain_yaw = 0.0f; // Gain setting for the yawn D-controller.

  float roll_level_adjust, pitch_level_adjust;
  float pid_error_temp;
  float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
  float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
  float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
  float temperatureC;
  float ax_mps2, ay_mps2, az_mps2;

  float gyroRateX;
  float gyroRateY;
  float gyroRateZ;
  float ax_g;
  float ay_g;
  float az_g;

  // Maximum output of the PID-controller Anti windup (+/-)
  // These values should be based on the practical maximum rates observed for stable control.
  // If the observed maximum rate during a maneuver (like a flip) is around 80 degrees per second, use that value.
  // The gyroscope maximum rate is typically higher (e.g., ±500 degrees per second), but we set a practical limit here.
  int pid_max_roll = 90;            // Practical maximum rate for roll in degrees per second
  int pid_max_pitch = pid_max_roll; // Practical maximum rate for pitch in degrees per second
  int pid_max_yaw = 90;             // Practical maximum rate for yaw in degrees per second

  int esc_1, esc_2, esc_3, esc_4;
  int start;

  int16_t raw_ax = 0, raw_ay = 0, raw_az = 0, raw_gx = 0, raw_gy = 0, raw_gz = 0;
  int16_t gyroXOffset, gyroYOffset, gyroZOffset, accXOffset, accYOffset, accZOffset;

  float acc_x, acc_y, acc_z, acc_total_vector;
  double gyro_pitch, gyro_roll, gyro_yaw;
  double pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
  float accRoll;
  float accPitch;
  bool isDebounceConditionMet;
  bool gyro_angles_set;
  bool auto_level = true; // Auto level on (true) or off (false)

  // Member functions
  void initialize();
  void initializeI2CBus();
  void performCalibration();
  void setupInputPins();
  void attachInterrupts();
  void attachESCPins();
  void armESCs();
  void allocatePWMTimers();
  void read_Controller();
  void level_flight(int &, int &, int &, int &);
  void motorControls();
  void calculate_pid();
  void mix_motors();
  void processIMUData(bool, bool);
  void write_motors();
  void startInitializationSequence();
  // float calculatePIDSetpoint(int channel, float level_adjust);
  // float calculatePIDSetpointForYaw(int channel_3, int channel_4);
  int computeESCValue(int, int, int, int);
  // float calculate_pid_component(float input, float setpoint, float &i_mem, float &last_d_error, float p_gain, float i_gain, float d_gain, float max_output, float dt);
  bool areMotorsOff();
  void print();
};

extern Flight_Controller flightController;

#endif // FLIGHT_CONTROLLER_h
