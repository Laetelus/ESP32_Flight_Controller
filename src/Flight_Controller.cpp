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

#define THROTTLE 36
#define YAW 39
#define ROLL 35
#define PITCH 34

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

  allocatePWMTimers();
  initializeI2CBus();
  performCalibration();
  setupInputPins();
  attachInterrupts();
  attachESCPins();
  armESCs();

  cal.printStoredCalibrationValues();

  // Load PID values from SPIFFS (if available)
  if (!ws.loadPIDValues())
  {
    Serial.println("No PID values loaded from SPIFFS. Using default values.");
  }

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

  auto readRegister = [](uint8_t reg) -> uint8_t
  {
    Wire.beginTransmission(0x68);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 1);
    return Wire.available() ? Wire.read() : 0xFF; // Return 0xFF if no data available
  };

  // Check if power management was set correctly
  if (readRegister(0x6B) != 0x00)
  {
    Serial.println("Failed to initialize clock source.");
  }

  // Set DLPF to reduce high-frequency noise
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Check if DLPF was set correctly
  if (readRegister(0x1A) != 0x05)
  {
    Serial.println("Failed to set DLPF.");
  }

  // Set accelerometer sensitivity to ±8g
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Check if accelerometer sensitivity was set correctly
  if (readRegister(0x1C) != 0x10)
  {
    Serial.println("Failed to set accelerometer sensitivity to ±8g.");
  }
  else
    Serial.println("Accelerometer sensitivity range set to ±8g. deg/s.");

  // Set gyroscope sensitivity to ±500 deg/s
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Check if gyroscope sensitivity was set correctly
  if (readRegister(0x1B) != 0x08)
  {
    Serial.println("Failed to set gyroscope sensitivity to ±500 deg/s.");
  }
  else
    Serial.println("Gyroscope sensitivity range set to ±500 deg/s.");

  Serial.println("Initialization and Configuration Complete. All systems go!");
}

void Flight_Controller::performCalibration()
{

// Calibration with or without EEPROM (only store EEPROM when data is satisfactory)
#ifdef USE_EEPROM
  EEPROM.begin(EEPROM_SIZE);
  if (!cal.loadCalibrationValues())
  {
    Serial.println("Calibration data not found in EEPROM. Calibrating...");
    // Blink LED to indicate calibration in progress
    for (int i = 0; i < 10; ++i)
    {
      digitalWrite(2, HIGH);
      delay(200);
      digitalWrite(2, LOW);
      delay(200);
    }
    processIMUData();
    calibrateMPU6050();
    // Turn off LED
    digitalWrite(2, LOW);
  }
  else
  {
    Serial.println("Calibration data found in EEPROM.");
  }
#else
  Serial.println("Calibrating without EEPROM...");
  digitalWrite(2, HIGH);
  processIMUData();
  calibrateMPU6050();
  digitalWrite(2, LOW);
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
  // Arm ESCs
  esc1.write(MIN_PULSE_WIDTH);
  esc2.write(MIN_PULSE_WIDTH);
  esc3.write(MIN_PULSE_WIDTH);
  esc4.write(MIN_PULSE_WIDTH);
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

void Flight_Controller::level_flight()
{

  gyro_roll_input = (gyro_roll_input * 0.7) + (gyro_roll * 0.3);
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (gyro_pitch * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (gyro_yaw * 0.3);

  // Gyro angle calculations
  // 0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611; // Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0000611;   // Calculate the traveled roll angle and add this to the angle_roll variable.

  // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066); // If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066); // If the IMU has yawed transfer the pitch angle to the roll angel.

  // Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); // Calculate the total accelerometer vector.

  // Accelerometer angle calculations

  if (abs(acc_y) < acc_total_vector)
  {
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;
  }
  if (abs(acc_x) < acc_total_vector)
  {
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;
  }

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004; // Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;    // Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15; // Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;   // Calculate the roll angle correction

  if (!auto_level)
  {                         // If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0; // Set the pitch angle correction to zero.
    roll_level_adjust = 0;  // Set the roll angle correcion to zero.
  }
}

void Flight_Controller::motorControls()
{
  int local_channel_1, local_channel_2, local_channel_3, local_channel_4;
  noInterrupts();
  local_channel_3 = receiver_input_channel_1;
  local_channel_4 = receiver_input_channel_4;
  local_channel_1 = receiver_input_channel_2;
  local_channel_2 = receiver_input_channel_3;
  interrupts();

  unsigned long currentTime = millis();

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
      isDebounceConditionMet = false;
    }
  }

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
      isDebounceConditionMet = false;
    }
  }

  if (start == 2 && local_channel_3 <= 1064 && local_channel_4 > 1976)
  {
    if (!isDebounceConditionMet)
    {
      lastDebounceTime = currentTime;
      isDebounceConditionMet = true;
    }
    else if ((currentTime - lastDebounceTime) > debounceDelay)
    {
      start = 0;
      isDebounceConditionMet = false;
    }
  }

  if (!(local_channel_3 < 1065 && local_channel_4 < 1050) && !(start == 1 && local_channel_3 < 1550 && local_channel_4 > 1450) && !(start == 2 && local_channel_3 <= 1064 && local_channel_4 > 1976))
  {
    isDebounceConditionMet = false;
  }

  // The PID set point in degrees per second is determined by the roll receiver input.
  // In the case of dividing by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s )
  pid_roll_setpoint = 0;
  if (local_channel_1 > 1508)
    pid_roll_setpoint = local_channel_1 - 1508;
  else if (local_channel_1 < 1492)
    pid_roll_setpoint = local_channel_1 - 1492;

  pid_roll_setpoint -= roll_level_adjust;
  pid_roll_setpoint /= 3.0;

  // The PID set point in degrees per second is determined by the pitch receiver input.
  // In the case of dividing by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  if (local_channel_2 > 1508)
    pid_pitch_setpoint = local_channel_2 - 1508;
  else if (local_channel_2 < 1492)
    pid_pitch_setpoint = local_channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;
  pid_pitch_setpoint /= 3.0;

  // The PID set point in degrees per second is determined by the yaw receiver input.
  // In the case of dividing by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  if (local_channel_3 > 1050)
  {
    if (local_channel_4 > 1508)
      pid_yaw_setpoint = (local_channel_4 - 1508) / 3.0;
    else if (local_channel_4 < 1492)
      pid_yaw_setpoint = (local_channel_4 - 1492) / 3.0;
  }

  // Serial.print("pid_roll_setpoint: ");
  // Serial.print(pid_roll_setpoint);
  // Serial.print(" pid_pitch_setpoint: ");
  // Serial.print(pid_pitch_setpoint);
  // Serial.print(" pid_yaw_setpoint: ");
  // Serial.println(pid_yaw_setpoint);

  calculate_pid();
}

// void Flight_Controller::calculate_pid()
// {
//   const float dt = 0.004; // Fixed loop time (250Hz)

//   pid_output_roll = calculate_pid_component(gyro_roll_input, pid_roll_setpoint, pid_i_mem_roll, pid_last_roll_d_error, pid_p_gain_roll, pid_i_gain_roll, pid_d_gain_roll, pid_max_roll, dt);
//   pid_output_pitch = calculate_pid_component(gyro_pitch_input, pid_pitch_setpoint, pid_i_mem_pitch, pid_last_pitch_d_error, pid_p_gain_pitch, pid_i_gain_pitch, pid_d_gain_pitch, pid_max_pitch, dt);
//   pid_output_yaw = calculate_pid_component(gyro_yaw_input, pid_yaw_setpoint, pid_i_mem_yaw, pid_last_yaw_d_error, pid_p_gain_yaw, pid_i_gain_yaw, pid_d_gain_yaw, pid_max_yaw, dt);
// }

void Flight_Controller::calculate_pid()
{
  // Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)
    pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)
    pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)
    pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)
    pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  // Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)
    pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)
    pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)
    pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)
    pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  // Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)
    pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)
    pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)
    pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)
    pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}
void Flight_Controller::mix_motors()
{
  int local_throttle;

  noInterrupts();
  local_throttle = receiver_input_channel_1; // Use local copy for throttle signal
  interrupts();

  if (start == 2)
  {                                                         // The motors are started.
    local_throttle = constrain(local_throttle, 1000, 1800); // Constrain and allow room for control at full throttle

    // Mixing algorithm for appropriate motors
    esc_1 = computeESCValue(local_throttle, -pid_output_pitch, pid_output_roll, pid_output_yaw);   // FR/CCW
    esc_2 = computeESCValue(local_throttle, -pid_output_pitch, -pid_output_roll, -pid_output_yaw); // FL/CW
    esc_3 = computeESCValue(local_throttle, pid_output_pitch, pid_output_roll, -pid_output_yaw);   // BR/CW
    esc_4 = computeESCValue(local_throttle, pid_output_pitch, -pid_output_roll, pid_output_yaw);   // BL/CCW
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
  esc_value = constrain(esc_value, 1100, 2000); // Ensure motors keep running and limit max pulse
  return esc_value;
}

void Flight_Controller::write_motors()
{
  // prevent ESCs from beeping before starting motors.
  if (start != 2)
  {
    armESCs();
  }
  else
  {
    esc1.writeMicroseconds(esc_1); // FR/CCW
    esc2.writeMicroseconds(esc_2); // FL/CW
    esc3.writeMicroseconds(esc_3); // BR/CW
    esc4.writeMicroseconds(esc_4); // BL/CCW
  }
}

void Flight_Controller::startInitializationSequence()
{
  start = 2;
  angle_pitch = angle_pitch_acc;
  angle_roll = angle_roll_acc;
  gyro_angles_set = true;

  // Reset the PID controllers for a bumpless start.
  pid_i_mem_roll = 0;
  pid_last_roll_d_error = 0;
  pid_i_mem_pitch = 0;
  pid_last_pitch_d_error = 0;
  pid_i_mem_yaw = 0;
  pid_last_yaw_d_error = 0;
}

// float Flight_Controller::calculatePIDSetpoint(int channel, float level_adjust)
// {
//   float setpoint = 0;
//   if (channel > 1508)
//     setpoint = (channel - 1508) / 3.0;
//   else if (channel < 1492)
//     setpoint = (channel - 1492) / 3.0;
//   setpoint -= level_adjust;

//   return setpoint;
// }

// float Flight_Controller::calculatePIDSetpointForYaw(int channel_3, int channel_4)
// {
//   if (channel_3 <= 1050)
//     return 0; // Do not yaw when turning off the motors
//   if (channel_4 > 1508)
//     return (channel_4 - 1508) / 3.0;
//   if (channel_4 < 1492)
//     return (channel_4 - 1492) / 3.0;
//   return 0;
// }

void Flight_Controller::calibrateMPU6050()
{

  long buff_ax = 0, buff_ay = 0,
       buff_az = 0, buff_gx = 0,
       buff_gy = 0, buff_gz = 0;

  // collect a couple samples
  const int num_samples = 2000;
  for (int i = 0; i < num_samples; i++)
  {
    processIMUData();
    buff_ax += acc_x;
    buff_ay += acc_y;
    buff_az += acc_z;
    buff_gx += gyro_roll;
    buff_gy += gyro_pitch;
    buff_gz += gyro_yaw;
    delay(2); // wait for sampling at 500Hz
  }

  // Average the values
  accXOffset = buff_ax / num_samples;
  accYOffset = buff_ay / num_samples;
  accZOffset = (buff_az / num_samples) - 4096; // considering 1g offset
  gyroXOffset = buff_gx / num_samples;
  gyroYOffset = buff_gy / num_samples;
  gyroZOffset = buff_gz / num_samples;
}

void Flight_Controller::processIMUData()
{
  int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;

  // Request 14 bytes starting from accelerometer's first register (0x3B)
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);            // Starting register for accelerometer data
  Wire.endTransmission(false); // End transmission but keep the connection active
  Wire.requestFrom(0x68, 14);  // Request a total of 14 bytes: 6 acc, 2 temp, 6 gyro

  // Read accelerometer data
  raw_ay = Wire.read() << 8 | Wire.read();
  raw_ax = Wire.read() << 8 | Wire.read();
  raw_az = Wire.read() << 8 | Wire.read();

  // Skip temperature bytes (unneeded)
  Wire.read();
  Wire.read();

  // Read gyroscope data
  raw_gx = Wire.read() << 8 | Wire.read();
  raw_gy = Wire.read() << 8 | Wire.read();
  raw_gz = Wire.read() << 8 | Wire.read();

  // Convert the accelerometer data to g's and gyroscope data to degrees per second
  acc_x = ((float)raw_ax / 4096) - accXOffset;
  acc_y = ((float)raw_ay / 4096) - accYOffset;
  acc_z = ((float)raw_az / 4096) - accZOffset;

  gyro_roll = ((float)raw_gx / 65.5) - gyroXOffset; // Convert to deg/s (assuming ±500°/s range and 65.5 LSB/deg/s sensitivity)
  gyro_pitch = ((float)raw_gy / 65.5) - gyroYOffset;
  gyro_yaw = ((float)raw_gz / 65.5) - gyroZOffset;

  // Optional: Invert signals if necessary
  bool invertAccX = true;
  bool invertAccY = true;
  bool invertGyroRoll = true;
  bool invertGyroYaw = true;

  if (invertAccX)
    acc_x = -acc_x;
  if (invertAccY)
    acc_y = -acc_y;
  if (invertGyroRoll)
    gyro_roll = -gyro_roll;
  if (invertGyroYaw)
    gyro_yaw = -gyro_yaw;

  // Assign to gyro inputs for PID calculation
  gyro_roll_input = gyro_roll;
  gyro_pitch_input = gyro_pitch;
  gyro_yaw_input = gyro_yaw;
}
