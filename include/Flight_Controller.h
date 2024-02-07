// FLIGHT_CONTROLLER_h
#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>
#include "Flight_Controller.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "SPIFFS.h"

class Flight_Controller
{
public:
    // initialization Sequence
    void initialize(); // Initialize the quad-copter
    void initializeI2CBus();
    void initializeGyroAndAccel();
    void performCalibration();
    void setupInputPins();
    void readCurrentTemperature();
    void attachInterrupts();
    void attachESCPins();
    void armESCs();
    void allocatePWMTimers();

    // Flight Controller
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

    // EEPROM
    void saveCalibrationValues();
    bool loadCalibrationValues();
    void printStoredCalibrationValues();
    void clearCalibrationData();

    // Web server for PID tuning
    void initSPIFFS();
    void initWiFi();
    void disconnect_wifi();
    void Handle_Server();
    void handleGetPID(AsyncWebServerRequest *request);
    void handleSetPID(AsyncWebServerRequest *request);
    void checkWiFiConnection();
    void fillPIDJson(DynamicJsonDocument &doc);
    String updatePIDFromRequest(AsyncWebServerRequest *request);
    bool savePIDValues();
    bool loadPIDValues();

private:
    // EEPROM memory layout for IMU
    #define EEPROM_SIZE 32

    AsyncWebServer server{80};
    AsyncEventSource events{"/events"};

    unsigned long lastDebounceTime = 0;     // Last time the input condition was met
    const unsigned long debounceDelay = 20; // Debounce period in milliseconds
    bool isDebounceConditionMet = false;    // Tracks if the condition was met

    // Web server and WiFi credentials
    const char *ssid = "Untrusted_Network";
    const char *password = "rapidcream878";
    int start;

    static constexpr int MIN_PULSE_LENGTH = 1000;
    static constexpr int MAX_PULSE_LENGTH = 2000;

    // ESC Pins
    static constexpr int esc_pin1 = 32; // FR/CCW
    static constexpr int esc_pin2 = 33; // FL/CW
    static constexpr int esc_pin3 = 25; // BR/CW
    static constexpr int esc_pin4 = 26; // BL/CCW

    // // PID Parameters
    float pid_p_gain_roll = 0.0f; // Gain setting for the roll P-controller
    float pid_i_gain_roll = 0.0f; // Gain setting for the roll I-controller
    float pid_d_gain_roll = 0.0f; // Gain setting for the roll D-controller
    int pid_max_roll = 400;       // Maximum output of the PID-controller (+/-)

    float pid_p_gain_pitch = pid_p_gain_roll; // Gain setting for the pitch P-controller.
    float pid_i_gain_pitch = pid_i_gain_roll; // Gain setting for the pitch I-controller.
    float pid_d_gain_pitch = pid_d_gain_roll; // Gain setting for the pitch D-controller.
    int pid_max_pitch = pid_max_roll;         // Maximum output of the PID-controller (+/-)

    float pid_p_gain_yaw = 0.0f; // Gain setting for the pitch P-controller. //
    float pid_i_gain_yaw = 0.0f; // Gain setting for the pitch I-controller. //
    float pid_d_gain_yaw = 0.0f; // Gain setting for the pitch D-controller.
    int pid_max_yaw = 400;       // Maximum output of the PID-controller (+/-)

    // Other variables
    int esc_1, esc_2, esc_3, esc_4;

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
    bool gyro_angles_set;
    bool auto_level = true; // Auto level on (true) or off (false)

    // initialize  units of g
    float ax_mps2 = 0.0f;
    float ay_mps2 = 0.0f;
    float az_mps2 = 0.0f;

    // MPU6050 instance
    MPU6050 accelgyro;

    // Servo motors
    Servo esc1, esc2, esc3, esc4;
};

#endif // FLIGHT_CONTROLLER_h