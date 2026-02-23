#include <Arduino.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include <Wire.h>
#include <EEPROM.h>
#include "Flight_Controller.h"
#include "Calibration.h"
#include "PID_Webserver.h"
#include "Kalman.h"
#include "ReceiverInput.h"
#include "Calibration.h"

Calibration cal; 


// #define USE_EEPROM

// Flight controller interface
void FC::initialize()
{

  Serial.begin(115200);
  pinMode(2, OUTPUT);
  
  //wifi init
  ws.initSPIFFS();
  WiFi.mode(WIFI_STA);
  ws.Wifi_task();

  // ─── Bring up I²C before touching the MPU ─────────────────────
  initializeI2CBus();
  setupInputPins();

  // we can just move this to setupInputPins 
  // attachInterrupts();

  //Perform MPU calibration. 
  cal.performCalibration();
  allocatePWMTimers();
  // finally Initialize and arm ESCs 
  Initialize_ESCs();

  // Why are we printing offsets here..? this should be used after knowing offset values for eeprom 
  // cal.printStoredCalibrationValues();

  // Comment #define EEPROM if clearing previously stored data
  // Once data is cleared. Ensure clearCalibrationData function is commented again
  // cal.clearCalibrationData();


  // we need to use this somewhere if we need to keep track of the loop time for dt 
  FC::filter_last_time = micros();

}

void FC::initializeI2CBus()
{

  Wire.begin();          // Initialize I2C communication
  Wire.setClock(400000); // Set I2C clock to 400kHz

  // Wake up MPU6050 and set clock source
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Power management register
  Wire.write(0x00); // Set clock source to internal 8 MHz oscillator
  Wire.endTransmission();

  // Set gyroscope sensitivity to ±500 deg/s
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Set accelerometer sensitivity to ±8g
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  // Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.beginTransmission(0x68);                                     
  Wire.write(0x1A);                                                  
  Wire.write(0x03);                                                 
  Wire.endTransmission();      

}


bool FC::areMotorsOff()
{
  return start != 2;
}

void FC::setupInputPins()
{
  pinMode(THROTTLE, INPUT);
  pinMode(YAW, INPUT);
  pinMode(PITCH, INPUT);
  pinMode(ROLL, INPUT);

  attachInterrupt(digitalPinToInterrupt(THROTTLE), handleThrottleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(YAW), handleYawInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROLL), handleRollInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PITCH), handlePitchInterrupt, CHANGE);
}

void FC::allocatePWMTimers()
{
  // Allow allocation of all timers. Consistent and accurate PWM.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
}

void FC::Initialize_ESCs()
{
  // Attach ESC pins
  esc1.attach(esc_pin1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // FR (Front Right)
  esc2.attach(esc_pin2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // BR (Back Right)
  esc3.attach(esc_pin3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // FL (Front Left)
  esc4.attach(esc_pin4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // BL (Back Left)

  // Normal arm procedure, setting to minimum throttle
  esc_1 = 1000;
  esc_2 = 1000;
  esc_3 = 1000;
  esc_4 = 1000;
  
  //Write the initial values to motors
  write_motors();

  // Keep sending the signal for a few seconds to ensure the ESCs are armed
  // unsigned long startTime = millis();
  // while (millis() - startTime < 5000)
  // { // Send for 5 seconds
  //   esc1.writeMicroseconds(1000);
  //   esc2.writeMicroseconds(1000);
  //   esc3.writeMicroseconds(1000);
  //   esc4.writeMicroseconds(1000);
  //   delay(50); // Adjust delay as necessary to maintain continuous signal
  // }

}


//wtf are you doing here??? reread brooks FC and redo this again please 
void FC::level_flight(int &local_channel_1, int &local_channel_2, int &local_channel_3, int &local_channel_4)
{

  const float angle_deadband      = 2.0f;   // ignore <2°
  const float level_adjust_factor = 0.10f;  // gentler pull

  float errorRoll = fabs(angle_roll)  > angle_deadband ? angle_roll  : 0;
  float errorPitch= fabs(angle_pitch) > angle_deadband ? angle_pitch : 0;

  roll_level_adjust  = errorRoll  * level_adjust_factor;
  pitch_level_adjust = errorPitch * level_adjust_factor;


  // If auto-level is disabled, set adjustments to zero
  if (!auto_level)
  {
    pitch_level_adjust = 0;
    roll_level_adjust = 0;
  }

  // Initialize desired rates
  float desired_rate_roll = 0;
  float desired_rate_pitch = 0;

  // Calculate desired rates based on user inputs for roll
  if (local_channel_1 > 1508)
  {
    desired_rate_roll = local_channel_1 - 1508;
  }
  else if (local_channel_1 < 1492)
  {
    desired_rate_roll = local_channel_1 - 1492;
  }

  // Calculate desired rates based on user inputs for pitch
  if (local_channel_2 > 1508)
  {
    desired_rate_pitch = local_channel_2 - 1508;
  }
  else if (local_channel_2 < 1492)
  {
    desired_rate_pitch = local_channel_2 - 1492;
  }

  // Adjust rates for level flight if auto-level is enabled
  if (auto_level)
  {
    desired_rate_roll -= roll_level_adjust;
    desired_rate_pitch -= pitch_level_adjust;
  }

  // The factor translates the receiver input into a rate in degrees per second
  float factor = 3.0; // Adjust this based on exact calculations

  // Set the PID setpoints based on desired rates and factor
  pid_roll_setpoint = desired_rate_roll / factor;
  pid_pitch_setpoint = desired_rate_pitch / factor;

  // Set the PID set point for yaw based on user inputs
  pid_yaw_setpoint = 0;
  if (local_channel_3 > 998)
  {
    if (local_channel_4 > 1606)
    {
      pid_yaw_setpoint = (local_channel_4 - 1606) / factor;
    }
    else if (local_channel_4 < 1492)
    {
      pid_yaw_setpoint = (local_channel_4 - 1492) / factor;
    }
  }
}

void FC::calculate_pid()
{
  // Roll calculations
  pid_error_temp = pid_roll_setpoint - gyro_roll_input; // Correct error calculation direction
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;


  // Constrain integral memory (anti-windup)
  if (pid_i_mem_roll > pid_max_roll)
    pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < -pid_max_roll)
    pid_i_mem_roll = -pid_max_roll;

  // PID output calculation including proportional, integral, and derivative terms
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);

  // Constrain PID output
  if (pid_output_roll > pid_max_roll)
    pid_output_roll = pid_max_roll;
  else if (pid_output_roll < -pid_max_roll)
    pid_output_roll = -pid_max_roll;

  pid_last_roll_d_error = pid_error_temp;

  // Pitch calculations
  pid_error_temp = pid_pitch_setpoint - gyro_pitch_input; // Correct error calculation direction
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;

  // Constrain integral memory (anti-windup)
  if (pid_i_mem_pitch > pid_max_pitch)
    pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < -pid_max_pitch)
    pid_i_mem_pitch = -pid_max_pitch;

  // PID output calculation including proportional, integral, and derivative terms
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);

  // Constrain PID output
  if (pid_output_pitch > pid_max_pitch)
    pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < -pid_max_pitch)
    pid_output_pitch = -pid_max_pitch;

  pid_last_pitch_d_error = pid_error_temp;

  // Yaw calculations
  pid_error_temp = pid_yaw_setpoint - gyro_yaw_input; // Correct error calculation direction
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;

  // Constrain integral memory (anti-windup)
  if (pid_i_mem_yaw > pid_max_yaw)
    pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < -pid_max_yaw)
    pid_i_mem_yaw = -pid_max_yaw;

  // PID output calculation including proportional, integral, and derivative terms
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);

  // Constrain PID output
  if (pid_output_yaw > pid_max_yaw)

    pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < -pid_max_yaw)
    pid_output_yaw = -pid_max_yaw;

  pid_last_yaw_d_error = pid_error_temp;

  // ─── Deadband to kill tiny noise ─────────────────────────────
  if (fabs(pid_output_roll)  < 1.0) pid_output_roll  = 0.0;
  if (fabs(pid_output_pitch) < 1.0) pid_output_pitch = 0.0;
  if (fabs(pid_output_yaw)   < 1.0) pid_output_yaw   = 0.0;
// ─────────────────────────────────────────────────────────────


}


void FC::processIMUData() {

  // Read raw IMU data from MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Start at the accelerometer data register
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14); // Request 14 bytes: 6 acc, 2 temp, 6 gyro

  // Read accelerometer data
  raw_ax = Wire.read() << 8 | Wire.read();
  raw_ay = Wire.read() << 8 | Wire.read();
  raw_az = Wire.read() << 8 | Wire.read();

  // Read temperature data (not used in this implementation)
  Wire.read() << 8 | Wire.read();

  // Read gyroscope data
  raw_gx = Wire.read() << 8 | Wire.read();
  raw_gy = Wire.read() << 8 | Wire.read();
  raw_gz = Wire.read() << 8 | Wire.read();

}

void FC::scale_IMU()
{
  // --- subtract OFFSETS IN RAW UNITS ---
  int16_t gx = raw_gx - gyroXOffset;
  int16_t gy = raw_gy - gyroYOffset;
  int16_t gz = raw_gz - gyroZOffset;

  int16_t ax = raw_ax - accXOffset;
  int16_t ay = raw_ay - accYOffset;
  int16_t az = raw_az - accZOffset;

  // --- Gyros scaled to deg/s ---
  gx_d = gx * (500.0f / 32768.0f);
  gy_d = gy * (500.0f / 32768.0f);
  gz_d = gz * (500.0f / 32768.0f);

  // --- Accel scaled to g ---
  ax_g = ax * (8.0f / 32768.0f);
  ay_g = ay * (8.0f / 32768.0f);
  az_g = az * (8.0f / 32768.0f);

  // Serial.println();
  // Serial.print("data_scaled.gyrox: "); Serial.print(data_scaled.gyroX);

}

void FC::calc_accel_angle()
{
  // Use atan2(ay, az) for roll in a flight controller.
  // The sqrt version is an approximation that only works when pitch is small.
  accRoll = atan2(ay_g, az_g) * RAD_TO_DEG;
  accPitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * RAD_TO_DEG;

}

void FC::motorControls()
{
  int local_channel_1, local_channel_2, local_channel_3, local_channel_4;
  noInterrupts();
  local_channel_3 = throttlePulseWidth;   // Throttle
  local_channel_4 = yawPulseWidth;        // Yaw
  local_channel_1 = rollPulseWidth;       // Roll
  local_channel_2 = pitchPulseWidth;      // Pitch
  interrupts();


  // Serial.print(rollPulseWidth); 
  // Serial.println(); 


  unsigned long currentTime = millis();

  // Start condition (start = 1)
  if (local_channel_3 < 1065 && local_channel_4 < 1050)
  {
    if (!isDebounceConditionMet)
    {
      lastDebounceTime = currentTime;
      isDebounceConditionMet = true;
    }
    else if ((currentTime - lastDebounceTime) > debounceDelay && start == 0)
    {
      start = 1;
      isDebounceConditionMet = false; // Reset for next condition
    }
  }

  // Transition to start = 2 (running), handled inside startInitializationSequence
  if (start == 1 && local_channel_3 < 1550 && local_channel_4 > 1450)
  {
    if (!isDebounceConditionMet)
    {
      lastDebounceTime = currentTime;
      isDebounceConditionMet = true;
    }
    else if ((currentTime - lastDebounceTime) > debounceDelay)
    {
      startInitializationSequence();
      isDebounceConditionMet = false; // Reset for next condition
    }
  }

  // Turn off motors (start = 0)
  if (start == 2 && local_channel_3 <= 1064 && local_channel_4 > 1976)
  {
    if (!isDebounceConditionMet)
    {
      lastDebounceTime = currentTime;
      isDebounceConditionMet = true;
    }
    else if ((currentTime - lastDebounceTime) > debounceDelay)
    {
      start = 0;                      // Turn off
      isDebounceConditionMet = false; // Reset for next condition
    }
  }

  // Reset debounce condition if none of the above conditions are met
  if (!(local_channel_3 < 1065 && local_channel_4 < 1050) &&
      !(start == 1 && local_channel_3 < 1550 && local_channel_4 > 1450) &&
      !(start == 2 && local_channel_3 <= 1064 && local_channel_4 > 1976))
  {
    isDebounceConditionMet = false;
  }

  // Perform level flight and PID calculations only if the motors are started
  if (start == 2)
  {
    level_flight(local_channel_1, local_channel_2, local_channel_3, local_channel_4);
    calculate_pid();
  }
}

void FC::mix_motors()
{
  int local_throttle;

  noInterrupts();
  local_throttle = throttlePulseWidth; // Use local copy for throttle signal
  interrupts();

  if (start == 2)
  {                                                         // The motors are started.
    local_throttle = constrain(local_throttle, 1000, 1800); // Constrain and allow room for control at full throttle

    esc_1 = computeESCValue(local_throttle, -pid_output_pitch, -pid_output_roll, pid_output_yaw); // FR/CCW
    esc_2 = computeESCValue(local_throttle, -pid_output_pitch, pid_output_roll, -pid_output_yaw); // FL/CW
    esc_3 = computeESCValue(local_throttle, pid_output_pitch, -pid_output_roll, -pid_output_yaw); // BR/CW
    esc_4 = computeESCValue(local_throttle, pid_output_pitch, pid_output_roll, pid_output_yaw);   // BL/CCW

    // // Current mixing algorithm matches my oriantation but
    // // Adjusted mixing algorithm for correct motor responses
    // // Yaw seems to be incorrect 
    // esc_1 = computeESCValue(local_throttle, -pid_output_pitch, -pid_output_roll, pid_output_yaw); // FR/CCW
    // esc_2 = computeESCValue(local_throttle, -pid_output_pitch, pid_output_roll, -pid_output_yaw); // FL/CW
    // esc_3 = computeESCValue(local_throttle, pid_output_pitch, -pid_output_roll, pid_output_yaw);  // BR/CW
    // esc_4 = computeESCValue(local_throttle, pid_output_pitch, pid_output_roll, -pid_output_yaw);  // BL/CCW

  }
  else
  {
    // If start is not 2, keep a 1000us pulse for all ESCs
    esc_1 = esc_2 = esc_3 = esc_4 = 1000;
  }

  write_motors(); 

}

int FC::computeESCValue(int throttle, int pitch, int roll, int yaw) {
  int v = throttle + pitch + roll + yaw;
  return constrain(v, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
}


void FC::write_motors(){

  esc1.writeMicroseconds(esc_1); // FR/CCW
  esc2.writeMicroseconds(esc_2); // FL/CW
  esc3.writeMicroseconds(esc_3); // BR/CW
  esc4.writeMicroseconds(esc_4); // BL/CCW
}

void FC::startInitializationSequence()
{

  start = 2;

  // processIMUData(); // triggers a Kalman filter update with the latest sensor data

  // Should set this as zero initially 
  // its good when we start on the ground to have a bumpless start. 
  // ─── NEW: LOCK IN CURRENT ATTITUDE AS ZERO-ERROR ───
  pid_roll_setpoint  = angle_roll;
  pid_pitch_setpoint = angle_pitch;
  pid_yaw_setpoint   = gyro_yaw_input; 
  
  // ───────────────────────────────────────────────────

  // reset PID state
  pid_i_mem_roll        = pid_last_roll_d_error  = 0;
  pid_i_mem_pitch       = pid_last_pitch_d_error = 0;
  pid_i_mem_yaw         = pid_last_yaw_d_error   = 0;

}

// void FC::processIMUData(bool applyOffsets, bool applyFiltering)
// {

//   Wire.beginTransmission(0x68);
//   Wire.write(0x1A);
//   Wire.write(0x05);
//   Wire.endTransmission();

//   Wire.beginTransmission(0x68);
//   Wire.write(0x1C);
//   Wire.write(0x10);
//   Wire.endTransmission();

//   // Communicate with MPU6050
//   Wire.beginTransmission(0x68);
//   Wire.write(0x3B); // Start at the accelerometer data register
//   Wire.endTransmission();
//   Wire.requestFrom(0x68, 6); // Request 14 bytes: 6 acc, 2 temp, 6 gyro

//   // Read accelerometer data
//   raw_ax = Wire.read() << 8 | Wire.read();
//   raw_ay = Wire.read() << 8 | Wire.read();
//   raw_az = Wire.read() << 8 | Wire.read();

//   // Start communication with MPU6050
//   Wire.beginTransmission(0x68);
//   Wire.write(0x43); // Start at the gyroscope data register
//   Wire.endTransmission();
//   Wire.requestFrom(0x68, 6); // Request 6 bytes for gyroscope data

//   // invert gyro x here and z here instead
//   //  Read gyroscope data
//   raw_gx = Wire.read() << 8 | Wire.read();
//   raw_gy = Wire.read() << 8 | Wire.read();
//   raw_gz = Wire.read() << 8 | Wire.read();

//   // Apply offsets if flag is set and convert raw data to physical units
//   if (applyOffsets)
//   {
//     raw_ax -= accXOffset;
//     raw_ay -= accYOffset;
//     raw_az -= accZOffset;
//     raw_gx -= gyroXOffset;
//     raw_gy -= gyroYOffset;
//     raw_gz -= gyroZOffset;
//     // print the offset to see if it works
//   }

//   // Convert gyroscope readings from raw to degrees per second
//   gyroRateX = (float)raw_gx / 65.5; // Gyro sensitivity for ±500 dps
//   gyroRateY = (float)raw_gy / 65.5;
//   gyroRateZ = (float)raw_gz / 65.5;

//   // Convert measurements to physical values
//   // Accel sensitivity for ±8g
//   ax_g = (float)raw_ax / 4096.0 + 0.01; // Do not forget to Modify each value
//   ay_g = (float)raw_ay / 4096.0 + 0.01;
//   az_g = (float)raw_az / 4096.0 + 0.001;

//   // conversion to degrees
//   accRoll = atan(ay_g / sqrt(ax_g * ax_g + az_g * az_g)) * RAD_TO_DEG;
//   accPitch = atan2(-ax_g, az_g) * RAD_TO_DEG;

//   // took from pratik
//   //  accRoll = atan(ay_g / sqrt(ax_g * ax_g + az_g * az_g)) * 1 / (3.142 / 180);
//   //  accPitch = -atan(ax_g / sqrt(ay_g * ay_g + az_g * az_g)) * 1 / (3.142 / 180);

//   // accRoll = atan2(ay_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180 / PI; // Roll calculation
//   // accPitch = -atan2(ax_g, az_g) * 180 / PI;                           // Pitch calculation

//   // Apply filtering if needed
//   if (applyFiltering)
//   {

//     // kalmanRoll.setQangle(0.0001);   // More smoothing, less responsiveness
//     // kalmanRoll.setQbias(0.001);     // Slower bias correction
//     // kalmanRoll.setRmeasure(0.05);   // Trust accelerometer less, smoother data

//     // kalmanPitch.setQangle(0.0001);
//     // kalmanPitch.setQbias(0.001);
//     // kalmanPitch.setRmeasure(0.05);

//     // Calculate delta time for the Kalman filter
//     unsigned long now = micros();
//     float dt = (now - filter_last_time) * 1e-6f;
//     filter_last_time = now;

//     // Update Kalman filter with new accelerometer and gyroscope data
//     angle_roll = kalmanRoll.getAngle(accRoll, gyroRateX, dt);
//     angle_pitch = kalmanPitch.getAngle(accPitch, gyroRateY, dt);

//     // // Complementary filter
//     // // 98% from integrated gyro angles, 2% from accelerometer
//     // angle_roll = 0.98 * (angle_roll + gyroRateX * dt) + 0.02 * accRoll;
//     // angle_pitch = 0.98 * (angle_pitch + gyroRateY * dt) + 0.02 * accPitch;

//     // angle_pitch = angle_pitch * 0.9996 + accPitch * 0.0004; // Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
//     // angle_roll = angle_roll * 0.9996 + accRoll * 0.0004;    // Correct the drift of the gyro roll angle with the accelerometer roll angle.
//   }

//   // // Set gyro angles equal to accelerometer angles when starting
//   // if (applyOffsets && applyFiltering)
//   // {
//   //   angle_roll = accRoll;
//   //   angle_pitch = accPitch;
//   // }

//   // Update gyro in deg for PID calculations
//   gyro_roll_input = gyroRateX;
//   gyro_pitch_input = gyroRateY;
//   gyro_yaw_input = gyroRateZ;
// }
