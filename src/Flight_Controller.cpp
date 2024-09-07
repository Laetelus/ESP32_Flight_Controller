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

Flight_Controller flightController;

// #define USE_EEPROM

// Global variables for pulse widths
volatile unsigned long lastRisingEdgeThrottle, throttlePulseWidth;
volatile unsigned long lastRisingEdgeYaw, yawPulseWidth;
volatile unsigned long lastRisingEdgeRoll, rollPulseWidth;
volatile unsigned long lastRisingEdgePitch, pitchPulseWidth;

// Receiver input variables
volatile unsigned long receiver_input_channel_3;
volatile unsigned long receiver_input_channel_4;
volatile unsigned long receiver_input_channel_1;
volatile unsigned long receiver_input_channel_2;

portMUX_TYPE muxThrottle = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxYaw = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxPitch = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxRoll = portMUX_INITIALIZER_UNLOCKED;

// Interrupt Service Routines for each channel
void IRAM_ATTR handleThrottleInterrupt()
{
  portENTER_CRITICAL_ISR(&muxThrottle);
  // GPIO port manipulation starting pg 49 in the esp32 tech manual
  if (GPIO.in1.val & (1ULL << (THROTTLE - 32)))
  {
    lastRisingEdgeThrottle = esp_timer_get_time();
  }
  else
  {
    throttlePulseWidth = esp_timer_get_time() - lastRisingEdgeThrottle;
  }
  portEXIT_CRITICAL_ISR(&muxThrottle);
}

void IRAM_ATTR handleYawInterrupt()
{
  portENTER_CRITICAL_ISR(&muxYaw);
  if (GPIO.in1.val & (1ULL << (YAW - 32)))
  {
    lastRisingEdgeYaw = esp_timer_get_time();
  }
  else
  {
    yawPulseWidth = esp_timer_get_time() - lastRisingEdgeYaw;
  }
  portEXIT_CRITICAL_ISR(&muxYaw);
}

void IRAM_ATTR handleRollInterrupt()
{
  portENTER_CRITICAL_ISR(&muxRoll);
  if (GPIO.in1.val & (1ULL << (ROLL - 32)))
  {
    lastRisingEdgeRoll = esp_timer_get_time();
  }
  else
  {
    rollPulseWidth = esp_timer_get_time() - lastRisingEdgeRoll;
  }
  portEXIT_CRITICAL_ISR(&muxRoll);
}

void IRAM_ATTR handlePitchInterrupt()
{
  portENTER_CRITICAL_ISR(&muxPitch);
  if (GPIO.in1.val & (1ULL << (PITCH - 32)))
  {
    lastRisingEdgePitch = esp_timer_get_time();
  }
  else
  {
    pitchPulseWidth = esp_timer_get_time() - lastRisingEdgePitch;
  }
  portEXIT_CRITICAL_ISR(&muxPitch);
}

// Flight controller interface
void Flight_Controller::initialize()
{
  Serial.begin(115200);
  pinMode(2, OUTPUT); // LED status

  attachESCPins();
  armESCs(); // Arm ESCs first
  allocatePWMTimers();
  initializeI2CBus();
  performCalibration();
  processIMUData(true, true);
  setupInputPins();
  attachInterrupts();

  cal.printStoredCalibrationValues();

  // Comment #define EEPROM if clearing previously stored data
  // Once data is cleared. Ensure clearCalibrationData function is commented again
  // cal.clearCalibrationData();
}

void Flight_Controller::initializeI2CBus()
{

  Wire.begin();          // Initialize I2C communication
  Wire.setClock(400000); // Set I2C clock to 400kHz

  // Wake up MPU6050 and set clock source
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Power management register
  Wire.write(0x00); // Set clock source to internal 8 MHz oscillator
  Wire.endTransmission();

  // auto readRegister = [](uint8_t reg) -> uint8_t
  // {
  //   Wire.beginTransmission(0x68);
  //   Wire.write(reg);
  //   Wire.endTransmission(false);
  //   Wire.requestFrom(0x68, 1);
  //   return Wire.available() ? Wire.read() : 0xFF; // Return 0xFF if no data available
  // };

  // // Check if power management was set correctly
  // if (readRegister(0x6B) != 0x00)
  // {
  //   Serial.println("Failed to initialize clock source.");
  // }

  // // Set DLPF to reduce high-frequency noise
  // Wire.beginTransmission(0x68);
  // Wire.write(0x1A);
  // Wire.write(0x05);
  // Wire.endTransmission();

  // // Check if DLPF was set correctly
  // if (readRegister(0x1A) != 0x05)
  // {
  //   Serial.println("Failed to set DLPF.");
  // }

  // // Set accelerometer sensitivity to ±8g
  // Wire.beginTransmission(0x68);
  // Wire.write(0x1C);
  // Wire.write(0x10);
  // Wire.endTransmission();

  // // Check if accelerometer sensitivity was set correctly
  // if (readRegister(0x1C) != 0x10)
  // {
  //   Serial.println("Failed to set accelerometer sensitivity to ±8g.");
  // }
  // else
  //   Serial.println("Accelerometer sensitivity range set to ±8g. deg/s.");

  // // Set gyroscope sensitivity to ±500 deg/s
  // Wire.beginTransmission(0x68);
  // Wire.write(0x1B);
  // Wire.write(0x08);
  // Wire.endTransmission();

  // // Check if gyroscope sensitivity was set correctly
  // if (readRegister(0x1B) != 0x08)
  // {
  //   Serial.println("Failed to set gyroscope sensitivity to ±500 deg/s.");
  // }
  // else
  //   Serial.println("Gyroscope sensitivity range set to ±500 deg/s.");

  // Serial.println("Initialization and Configuration Complete.");
}

void Flight_Controller::performCalibration()
{
  unsigned long cal_time = 10000; // Run for 10 seconds
  long buff_ax = 0, buff_ay = 0,
       buff_az = 0, buff_gx = 0,
       buff_gy = 0, buff_gz = 0;
  int sampleCount = 0;

#ifdef USE_EEPROM
  EEPROM.begin(EEPROM_SIZE);
  if (!cal.loadCalibrationValues())
  {
    Serial.println("Calibration data not found in EEPROM. Calibrating...");
    // Blink LED to indicate calibration is in progress
    digitalWrite(2, HIGH); // Turn on LED to indicate start of calibration

    unsigned long startTime = millis();
    while (millis() - startTime < cal_time)
    {
      processIMUData(false, false); // Collect raw data for calibration period
      buff_ax += raw_ax;
      buff_ay += raw_ay;
      buff_az += raw_az - 4096;

      buff_gx += raw_gx;
      buff_gy += raw_gy;
      buff_gz += raw_gz;

      sampleCount++; // Increment sample count
      delay(2);      // Delay to maintain sampling rate
    }

    // Use the actual counted samples for offset calculation
    accXOffset = buff_ax / sampleCount;
    accYOffset = buff_ay / sampleCount;
    accZOffset = buff_az / sampleCount;

    gyroXOffset = buff_gx / sampleCount;
    gyroYOffset = buff_gy / sampleCount;
    gyroZOffset = buff_gz / sampleCount;

    cal.saveCalibrationValues();
    digitalWrite(2, LOW); // Turn off LED after calibration.
  }
  else
  {
    Serial.println("Calibration data found in EEPROM.");
  }
#else
  Serial.println("Calibrating without EEPROM...");
  digitalWrite(2, HIGH); // Turn on LED to indicate start of calibration

  unsigned long startTime = millis();
  while (millis() - startTime < cal_time)
  {
    processIMUData(false, false); // Collect raw data for calibration period
    buff_ax += raw_ax;
    buff_ay += raw_ay;
    buff_az += raw_az - 4096;

    buff_gx += raw_gx;
    buff_gy += raw_gy;
    buff_gz += raw_gz;

    sampleCount++; // Increment sample count
    delay(2);      // Delay to maintain sampling rate
  }

  // Use the actual counted samples for offset calculation
  accXOffset = buff_ax / sampleCount;
  accYOffset = buff_ay / sampleCount;
  accZOffset = buff_az / sampleCount;

  gyroXOffset = buff_gx / sampleCount;
  gyroYOffset = buff_gy / sampleCount;
  gyroZOffset = buff_gz / sampleCount;

  digitalWrite(2, LOW); // Turn off LED after calibration.

  // Print calculated offsets for debugging
  Serial.println("Calibration Complete.");
  Serial.println("Calculated Offsets:");
  Serial.print("Acc X Offset: ");
  Serial.println(accXOffset);
  Serial.print("Acc Y Offset: ");
  Serial.println(accYOffset);
  Serial.print("Acc Z Offset: ");
  Serial.println(accZOffset);
  Serial.print("Gyro X Offset: ");
  Serial.println(gyroXOffset);
  Serial.print("Gyro Y Offset: ");
  Serial.println(gyroYOffset);
  Serial.print("Gyro Z Offset: ");
  Serial.println(gyroZOffset);

#endif
}

bool Flight_Controller::areMotorsOff()
{
  return start != 2;
}

void Flight_Controller::setupInputPins()
{
  pinMode(THROTTLE, INPUT);
  pinMode(YAW, INPUT);
  pinMode(PITCH, INPUT);
  pinMode(ROLL, INPUT);
}

void Flight_Controller::allocatePWMTimers()
{
  // Allow allocation of all timers. Consistent and accurate PWM.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
}

void Flight_Controller::attachInterrupts()
{
  // Attach the interrupts
  attachInterrupt(digitalPinToInterrupt(THROTTLE), handleThrottleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(YAW), handleYawInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROLL), handleRollInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PITCH), handlePitchInterrupt, CHANGE);
}

void Flight_Controller::attachESCPins()
{
  // Attach ESC pins
  esc1.attach(esc_pin1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // FR (Front Right)
  esc2.attach(esc_pin2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // BR (Back Right)
  esc3.attach(esc_pin3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // FL (Front Left)
  esc4.attach(esc_pin4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // BL (Back Left)
}

void Flight_Controller::armESCs()
{

  // Normal arm procedure, setting to minimum throttle
  esc_1 = 1000;
  esc_2 = 1000;
  esc_3 = 1000;
  esc_4 = 1000;
  write_motors();

  // // Keep sending the signal for a few seconds to ensure the ESCs are armed
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

void Flight_Controller::read_Controller()
{
  noInterrupts();                                // Disable interrupts to read the shared variables safely
  receiver_input_channel_1 = throttlePulseWidth; // Throttle
  receiver_input_channel_4 = yawPulseWidth;      // Yaw
  receiver_input_channel_2 = rollPulseWidth;     // Roll
  receiver_input_channel_3 = pitchPulseWidth;    // Pitch
  interrupts();                                  // Re-enable interrupts
}

void Flight_Controller::level_flight(int& local_channel_1, int& local_channel_2, int& local_channel_3, int& local_channel_4)
{
  const float level_adjust_factor = 0.15;

  // Calculate level adjustments based on current angles
  pitch_level_adjust = angle_pitch * level_adjust_factor;
  roll_level_adjust = angle_roll * level_adjust_factor;

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

//// #2 problematic we convert to deg twice
// void Flight_Controller::level_flight(int *local_channel_1, int *local_channel_2, int *local_channel_3, int *local_channel_4)
// {
//   // This factor determines how aggressively the drone attempts to return to a level position.
//   const float level_adjust_factor = 0.15;

//   // Calculate level adjustments
//   pitch_level_adjust = angle_pitch * level_adjust_factor; // Adjust angle here as well if need
//   roll_level_adjust = angle_roll * level_adjust_factor;   // Adjust angle here as well if need

//   // Calculate desired rates
//   float desired_rate_roll = (*local_channel_1 - 1503) * level_adjust_factor;  // Adjust angle here as well if need
//   float desired_rate_pitch = (*local_channel_2 - 1503) * level_adjust_factor; // Adjust angle here as well if need

//   // Update PID setpoints based on angle errors
//   pid_roll_setpoint = desired_rate_roll - angle_roll;
//   pid_pitch_setpoint = desired_rate_pitch - angle_pitch;

//   // Compute angle corrections if auto-level is enabled
//   if (auto_level)
//   {
//     pid_roll_setpoint -= roll_level_adjust;
//     pid_pitch_setpoint -= pitch_level_adjust;
//   }

//   // The PID set point in degrees per second is determined by the roll receiver input.
//   // In the case of dividing by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s )

//   // Set factor for converting receiver input to degrees per second
//   // The receiver input range is from 1000 to 2000 microseconds, with 1500 as the midpoint.
//   // The input deviation from 1500 microseconds represents the desired rate of change for roll, pitch, and yaw.
//   // If you want a maximum angular velocity of 80 degrees per second, calculate the factor as follows:

//   // Calculation example:
//   // 1. Determine the input range deviation: 2000 - 1500 = 500 microseconds (similarly, 1500 - 1000 = 500 microseconds)
//   // 2. Desired maximum rate: 80 degrees per second
//   // 3. Factor = Input range deviation / Desired max rate
//   // 4. Factor = 500 microseconds / "80" degrees per second
//   // 5. Factor = "6.25"
//   float factor = 6.25; // Adjust base on observed practical maximum rate

//   pid_roll_setpoint = 0;
//   if (*local_channel_1 > 1508)
//     pid_roll_setpoint = *local_channel_1 - 1508;
//   else if (*local_channel_1 < 1492)
//     pid_roll_setpoint = *local_channel_1 - 1492;

//   pid_roll_setpoint -= roll_level_adjust; // Subtract the angle correction from the standardized receiver roll input value.
//   pid_roll_setpoint /= factor;            // Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

//   // The PID set point in degrees per second is determined by the pitch receiver input.
//   // In the case of dividing by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
//   pid_pitch_setpoint = 0;
//   if (*local_channel_2 > 1508)
//     pid_pitch_setpoint = *local_channel_2 - 1508;
//   else if (*local_channel_2 < 1492)
//     pid_pitch_setpoint = *local_channel_2 - 1492;

//   pid_pitch_setpoint -= pitch_level_adjust; // Subtract the angle correction from the standardized receiver pitch input value.
//   pid_pitch_setpoint /= factor;             // Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

//   // The PID set point in degrees per second is determined by the yaw receiver input.
//   // In the case of dividing by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
//   pid_yaw_setpoint = 0;
//   if (*local_channel_3 > 998)
//   {
//     if (*local_channel_4 > 1606)
//       pid_yaw_setpoint = (*local_channel_4 - 1606) / factor;
//     else if (*local_channel_4 < 1492)
//       pid_yaw_setpoint = (*local_channel_4 - 1492) / factor;
//   }
// }

void Flight_Controller::calculate_pid()
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
}


void Flight_Controller::motorControls()
{
  int local_channel_1, local_channel_2, local_channel_3, local_channel_4;
  noInterrupts();
  local_channel_3 = receiver_input_channel_1; // Throttle
  local_channel_4 = receiver_input_channel_4; // Yaw
  local_channel_1 = receiver_input_channel_2; // Roll
  local_channel_2 = receiver_input_channel_3; // Pitch
  interrupts();

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

// //pid 2 incorrect?
// void Flight_Controller::calculate_pid()
// {
//   // Roll calculations
//   pid_error_temp = gyro_roll_input - pid_roll_setpoint;
//   pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;

//   if (pid_i_mem_roll > pid_max_roll)
//     pid_i_mem_roll = pid_max_roll;
//   else if (pid_i_mem_roll < pid_max_roll * -1)
//     pid_i_mem_roll = pid_max_roll * -1;

//   pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);

//   if (pid_output_roll > pid_max_roll)
//     pid_output_roll = pid_max_roll;
//   else if (pid_output_roll < pid_max_roll * -1)
//     pid_output_roll = pid_max_roll * -1;

//   pid_last_roll_d_error = pid_error_temp;

//   // Pitch calculations
//   pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
//   pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
//   if (pid_i_mem_pitch > pid_max_pitch)
//     pid_i_mem_pitch = pid_max_pitch;
//   else if (pid_i_mem_pitch < pid_max_pitch * -1)
//     pid_i_mem_pitch = pid_max_pitch * -1;

//   pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
//   if (pid_output_pitch > pid_max_pitch)
//     pid_output_pitch = pid_max_pitch;
//   else if (pid_output_pitch < pid_max_pitch * -1)
//     pid_output_pitch = pid_max_pitch * -1;

//   pid_last_pitch_d_error = pid_error_temp;

//   // Yaw calculations
//   pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
//   pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
//   if (pid_i_mem_yaw > pid_max_yaw)
//     pid_i_mem_yaw = pid_max_yaw;
//   else if (pid_i_mem_yaw < pid_max_yaw * -1)
//     pid_i_mem_yaw = pid_max_yaw * -1;

//   pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
//   if (pid_output_yaw > pid_max_yaw)
//     pid_output_yaw = pid_max_yaw;
//   else if (pid_output_yaw < pid_max_yaw * -1)
//     pid_output_yaw = pid_max_yaw * -1;

//   pid_last_yaw_d_error = pid_error_temp;
// }

void Flight_Controller::mix_motors()
{
  int local_throttle;

  noInterrupts();
  local_throttle = receiver_input_channel_1; // Use local copy for throttle signal
  interrupts();

  if (start == 2)
  {                                                         // The motors are started.
    local_throttle = constrain(local_throttle, 1000, 1800); // Constrain and allow room for control at full throttle

    // Current mixing algorithm matches my oriantation but
    // Adjusted mixing algorithm for correct motor responses
    esc_1 = computeESCValue(local_throttle, -pid_output_pitch, -pid_output_roll, pid_output_yaw); // FR/CCW
    esc_2 = computeESCValue(local_throttle, -pid_output_pitch, pid_output_roll, -pid_output_yaw); // FL/CW
    esc_3 = computeESCValue(local_throttle, pid_output_pitch, -pid_output_roll, pid_output_yaw);  // BR/CW
    esc_4 = computeESCValue(local_throttle, pid_output_pitch, pid_output_roll, -pid_output_yaw);  // BL/CCW
  }
  else
  {
    // If start is not 2, keep a 1000us pulse for all ESCs
    esc_1 = esc_2 = esc_3 = esc_4 = 1000;
  }
}

int Flight_Controller::computeESCValue(int throttle, int pitch, int roll, int yaw)
{
  int esc_value = throttle + pitch + roll + yaw;
  esc_value = map(esc_value, 1000, 2000, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  esc_value = constrain(esc_value, 1100, 1800); // Ensure motors keep running and limit max pulse
  return esc_value;
}

void Flight_Controller::write_motors()
{

  esc1.writeMicroseconds(esc_1); // FR/CCW
  esc2.writeMicroseconds(esc_2); // FL/CW
  esc3.writeMicroseconds(esc_3); // BR/CW
  esc4.writeMicroseconds(esc_4); // BL/CCW
}

void Flight_Controller::startInitializationSequence()
{
  start = 2;

  // Use the latest Kalman filter outputs to set initial angles
  // Ensure the latest sensor readings are processed just before starting
  processIMUData(true, true); // triggers a Kalman filter update with the latest sensor data

  gyro_angles_set = true;

  // Reset the PID controllers for a bumpless start
  pid_i_mem_roll = 0;
  pid_last_roll_d_error = 0;
  pid_i_mem_pitch = 0;
  pid_last_pitch_d_error = 0;
  pid_i_mem_yaw = 0;
  pid_last_yaw_d_error = 0;
}

void Flight_Controller::processIMUData(bool applyOffsets, bool applyFiltering)
{

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Communicate with MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Start at the accelerometer data register
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6); // Request 14 bytes: 6 acc, 2 temp, 6 gyro

  // Read accelerometer data
  raw_ax = Wire.read() << 8 | Wire.read();
  raw_ay = Wire.read() << 8 | Wire.read();
  raw_az = Wire.read() << 8 | Wire.read();

  // Start communication with MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x43); // Start at the gyroscope data register
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6); // Request 6 bytes for gyroscope data

  // invert gyro x here and z here instead
  //  Read gyroscope data
  raw_gx = Wire.read() << 8 | Wire.read();
  raw_gy = Wire.read() << 8 | Wire.read();
  raw_gz = Wire.read() << 8 | Wire.read();

  // Apply offsets if flag is set and convert raw data to physical units
  if (applyOffsets)
  {
    raw_ax -= accXOffset;
    raw_ay -= accYOffset;
    raw_az -= accZOffset;
    raw_gx -= gyroXOffset;
    raw_gy -= gyroYOffset;
    raw_gz -= gyroZOffset;
    // print the offset to see if it works
  }

  // Convert gyroscope readings from raw to degrees per second
  gyroRateX = (float)raw_gx / 65.5; // Gyro sensitivity for ±500 dps
  gyroRateY = (float)raw_gy / 65.5;
  gyroRateZ = (float)raw_gz / 65.5;

  // Convert measurements to physical values
  // Accel sensitivity for ±8g
  ax_g = (float)raw_ax / 4096.0 + 0.01; // Do not forget to Modify each value
  ay_g = (float)raw_ay / 4096.0 + 0.01;
  az_g = (float)raw_az / 4096.0 + 0.001;

  // conversion to degrees
  accRoll = atan(ay_g / sqrt(ax_g * ax_g + az_g * az_g)) * RAD_TO_DEG;
  accPitch = atan2(-ax_g, az_g) * RAD_TO_DEG;

  // took from pratik
  //  accRoll = atan(ay_g / sqrt(ax_g * ax_g + az_g * az_g)) * 1 / (3.142 / 180);
  //  accPitch = -atan(ax_g / sqrt(ay_g * ay_g + az_g * az_g)) * 1 / (3.142 / 180);

  // accRoll = atan2(ay_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180 / PI; // Roll calculation
  // accPitch = -atan2(ax_g, az_g) * 180 / PI;                           // Pitch calculation

  // Apply filtering if needed
  if (applyFiltering)
  {
    // Calculate delta time for the Kalman filter
    static unsigned long lastTime = 0;
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0f; // Convert microseconds to seconds
    lastTime = currentTime;

    // Update Kalman filter with new accelerometer and gyroscope data
    angle_roll = kalmanRoll.getAngle(accRoll, gyroRateX, dt);
    angle_pitch = kalmanPitch.getAngle(accPitch, gyroRateY, dt);

    // Complementary filter
    // // 98% from integrated gyro angles, 2% from accelerometer
    angle_roll = 0.98 * (angle_roll + gyroRateX * dt) + 0.02 * accRoll;
    angle_pitch = 0.98 * (angle_pitch + gyroRateY * dt) + 0.02 * accPitch;

    // angle_pitch = angle_pitch * 0.9996 + accPitch * 0.0004; // Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    // angle_roll = angle_roll * 0.9996 + accRoll * 0.0004;    // Correct the drift of the gyro roll angle with the accelerometer roll angle.
  }

  // Set gyro angles equal to accelerometer angles when starting
  if (applyOffsets && applyFiltering)
  {
    angle_roll = accRoll;
    angle_pitch = accPitch;
  }

  // Update gyro in deg for PID calculations
  gyro_roll_input = gyroRateX;
  gyro_pitch_input = gyroRateY;
  gyro_yaw_input = gyroRateZ;
}
