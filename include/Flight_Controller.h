#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Calibration.h"
#include "PID_Webserver.h"
#include <Wire.h>

struct Flight_Controller {

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
    const unsigned long debounceDelay = 20; // This could also be made static constexpr if it does not change

    float pid_p_gain_roll, pid_i_gain_roll, pid_d_gain_roll;
    float pid_p_gain_pitch, pid_i_gain_pitch, pid_d_gain_pitch;
    float pid_p_gain_yaw, pid_i_gain_yaw, pid_d_gain_yaw;
    float roll_level_adjust, pitch_level_adjust;
    float pid_error_temp;
    float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
    float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
    float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
    float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
    float gyroXOffset, gyroYOffset, gyroZOffset, accXOffset, accYOffset, accZOffset;
    float temperatureC;
    float ax_mps2, ay_mps2, az_mps2;
    int pid_max_roll, pid_max_pitch, pid_max_yaw;
    int esc_1, esc_2, esc_3, esc_4;
    int start;

    int16_t acc_x, acc_y, acc_z, acc_total_vector;
    int16_t gyro_pitch, gyro_roll, gyro_yaw;
    int16_t temp;

    bool isDebounceConditionMet;
    bool gyro_angles_set;
    bool auto_level;

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

};

#endif // FLIGHT_CONTROLLER_h



// // FLIGHT_CONTROLLER_h
// #ifndef FLIGHT_CONTROLLER_H
// #define FLIGHT_CONTROLLER_H

// #include <Arduino.h>
// #include <ESP32Servo.h>
// #include "I2Cdev.h"
// #include "MPU6050.h"
// #include <EEPROM.h>
// #include "Flight_Controller.h"
// #include <WiFi.h>
// #include <ESPAsyncWebServer.h>

// #include <AsyncTCP.h>
// #include <ArduinoJson.h>
// #include <Wire.h>
// #include "SPIFFS.h"

// class Flight_Controller
// {
// public:

// Flight_Controller()
// :
//   lastDebounceTime(0), // Initialize debounce time tracking

//   pid_p_gain_roll(0.0f), // Initialize PID gains to default values
//   pid_i_gain_roll(0.0f),
//   pid_d_gain_roll(0.0f),
//   pid_max_roll(400), // Default max PID output

//   pid_p_gain_pitch(0.0f), // Repeating for pitch and yaw 
//   pid_i_gain_pitch(0.0f),
//   pid_d_gain_pitch(0.0f),
//   pid_max_pitch(400),

//   pid_p_gain_yaw(0.0f),
//   pid_i_gain_yaw(0.0f),
//   pid_d_gain_yaw(0.0f),
//   pid_max_yaw(400),

//   events("/events"), // Initialize event source with URL path
//   isDebounceConditionMet(false), // Initialize boolean flags
//   gyro_angles_set(false),
//   auto_level(true) // Default to auto leveling on

// {

//   Serial.begin(250000);
//   pinMode(2, OUTPUT); // LED status

//   allocatePWMTimers();
//   initializeI2CBus();
//   initializeGyroAndAccel();
//   readCurrentTemperature();
//   performCalibration();
//   setupInputPins();
//   attachInterrupts();
//   attachESCPins();
//   armESCs();
//   printStoredCalibrationValues();

//   // Load PID values from SPIFFS (if available)
//   if (!loadPIDValues())
//   {
//     Serial.println("No PID values loaded from SPIFFS. Using default values.");
//   }

//   // Comment #define EEPROM if clearing previously stored data
//   // Once data is cleared. Ensure clearCalibrationData function is commented again
//   // clearCalibrationData();
    
// }
//     // initialization Sequence
//     //void initialize(); // Initialize the quad-copter
    
//     void initializeI2CBus();
//     void initializeGyroAndAccel();
//     void performCalibration();
//     void setupInputPins();
//     void readCurrentTemperature();
//     void attachInterrupts();
//     void attachESCPins();
//     void armESCs();
//     void allocatePWMTimers();

   
//     void read_Controller();
//     void level_flight();
//     void motorControls();
//     void calculate_pid();
//     void mix_motors();
//     void calibrateMPU6050();
//     int16_t applyDeadzone(int16_t, int16_t);
//     void processIMUData();
//     void readGyroData();
//     void write_motors();
//     void startInitializationSequence();
//     float calculatePIDSetpoint(int channel, float level_adjust);
//     float calculatePIDSetpointForYaw(int channel_3, int channel_4);
//     int computeESCValue(int throttle, int pitch, int roll, int yaw);
//     float calculate_pid_component(float input, float setpoint, float &i_mem, float &last_d_error, float p_gain, float i_gain, float d_gain, float max_output);
//     bool areMotorsOff();
//     void print();

//     // EEPROM
//     void saveCalibrationValues();
//     bool loadCalibrationValues();
//     void printStoredCalibrationValues();
//     void clearCalibrationData();

//     // Web server for PID tuning
//     void initSPIFFS();
//     void initWiFi();
//     void disconnect_wifi();
//     void Handle_Server();
//     void handleGetPID(AsyncWebServerRequest *request);
//     void handleSetPID(AsyncWebServerRequest *request);
//     void checkWiFiConnection();
//     void fillPIDJson(DynamicJsonDocument &doc);
//     String updatePIDFromRequest(AsyncWebServerRequest *request);
//     bool savePIDValues();
//     bool loadPIDValues();

// private:

//     static constexpr int EEPROM_SIZE = 32;
//     static constexpr int MIN_PULSE_LENGTH = 1000;
//     static constexpr int MAX_PULSE_LENGTH = 2000;
//     static constexpr int esc_pin1 = 32; // FR/CCW
//     static constexpr int esc_pin2 = 33; // FL/CW
//     static constexpr int esc_pin3 = 25; // BR/CW
//     static constexpr int esc_pin4 = 26; // BL/CCW

//     MPU6050 accelgyro;
//     AsyncWebServer server{80};
//     AsyncEventSource events{"/events"};
//     Servo esc1, esc2, esc3, esc4;

//     float pid_p_gain_roll, pid_i_gain_roll, pid_d_gain_roll;
//     float pid_p_gain_pitch, pid_i_gain_pitch, pid_d_gain_pitch;
//     float pid_p_gain_yaw, pid_i_gain_yaw, pid_d_gain_yaw;
//     float roll_level_adjust, pitch_level_adjust;
//     float pid_error_temp;
//     float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
//     float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
//     float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
//     float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
//     float gyroXOffset, gyroYOffset, gyroZOffset, accXOffset, accYOffset, accZOffset;
//     float temperatureC;
//     float ax_mps2, ay_mps2, az_mps2;

//     int pid_max_roll, pid_max_pitch, pid_max_yaw;
//     int esc_1, esc_2, esc_3, esc_4;
//     int start;
//     int16_t acc_x, acc_y, acc_z, acc_total_vector;
//     int16_t gyro_pitch, gyro_roll, gyro_yaw;
//     int16_t temp;

//     unsigned long lastDebounceTime;
//     const unsigned long debounceDelay = 20; // This could also be made static constexpr if it does not change
//     bool isDebounceConditionMet;
//     bool gyro_angles_set;
//     bool auto_level;

//     const char *ssid = "Untrusted_Network";
//     const char *password = "rapidcream878";
// };

// #endif // FLIGHT_CONTROLLER_h