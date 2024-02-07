#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>
#include <EEPROM.h>
#include "Flight_Controller.h"

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
  if (digitalRead(THROTTLE) == HIGH)
  {
    lastRisingEdgeThrottle = micros();
  }
  else
  {
    throttlePulseWidth = micros() - lastRisingEdgeThrottle;
  }
  portEXIT_CRITICAL_ISR(&muxThrottle);
}

void IRAM_ATTR handleYawInterrupt()
{
  portENTER_CRITICAL_ISR(&muxYaw);
  if (digitalRead(YAW) == HIGH)
  {
    lastRisingEdgeYaw = micros();
  }
  else
  {
    yawPulseWidth = micros() - lastRisingEdgeYaw;
  }
  portEXIT_CRITICAL_ISR(&muxYaw);
}

void IRAM_ATTR handleRollInterrupt()
{
  portENTER_CRITICAL_ISR(&muxRoll);
  if (digitalRead(ROLL) == HIGH)
  {
    lastRisingEdgeRoll = micros();
  }
  else
  {
    rollPulseWidth = micros() - lastRisingEdgeRoll;
  }
  portEXIT_CRITICAL_ISR(&muxRoll);
}

void IRAM_ATTR handlePitchInterrupt()
{
  portENTER_CRITICAL_ISR(&muxPitch);
  if (digitalRead(PITCH) == HIGH)
  {
    lastRisingEdgePitch = micros();
  }
  else
  {
    pitchPulseWidth = micros() - lastRisingEdgePitch;
  }
  portEXIT_CRITICAL_ISR(&muxPitch);
}

// Flight controller interface
void Flight_Controller::initialize()
{

  // Allow allocation of all timers. Consistent and accurate PWM.
  Serial.begin(250000);
  pinMode(2, OUTPUT); // LED status

  allocatePWMTimers();
  initializeI2CBus();
  initializeGyroAndAccel();
  readCurrentTemperature();
  performCalibration();
  setupInputPins();
  attachInterrupts();
  attachESCPins();
  armESCs();
  printStoredCalibrationValues();

  // Load PID values from SPIFFS (if available)
  if (!loadPIDValues())
  {
    Serial.println("No PID values loaded from SPIFFS. Using default values.");
  }

  // Comment #define EEPROM if clearing previously stored data
  // Once data is cleared. Ensure clearCalibrationData function is commented again
  // clearCalibrationData();
}

void Flight_Controller::initializeI2CBus()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device. This also initialises the gyros and accelerometers
  Serial.println();
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  // Verify gyroscope range
  uint8_t gyroRange = accelgyro.getFullScaleGyroRange();
  Serial.println(gyroRange == MPU6050_IMU::MPU6050_GYRO_FS_250 ? "Gyroscope range verified as ±250°/s" : "Gyroscope range verification failed");
}

void Flight_Controller::initializeGyroAndAccel()
{
  // Verify accelerometer range
  uint8_t accelRange = accelgyro.getFullScaleAccelRange();
  Serial.println(accelRange == MPU6050_IMU::MPU6050_GYRO_FS_250 ? "Accelerometer range verified as ±2g" : "Accelerometer range verification failed");

// verify connection
#ifdef I2CDEV_IMPLEMENTATION
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "accelgyro6050 connection successful" : "accelgyro6050 connection failed");
#endif
}

void Flight_Controller::readCurrentTemperature()
{
  // Read current temperature
  temp = accelgyro.getTemperature();
  temperatureC = float(temp) / 340.00 + 36.53;
  Serial.print("Current temperature: ");
  Serial.println(temperatureC);
}

void Flight_Controller::performCalibration()
{
// Calibration with or without EEPROM (only store EEPROM when data is satisfactory)
#ifdef USE_EEPROM
  EEPROM.begin(EEPROM_CALIBRATION_SIZE);
  if (!loadCalibrationValues())
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
    readGyroData();
    processIMUData();
    calibrateMPU6050();
    // Check if the temperature is within the acceptable range
    int minTemp = -40; // Replace with your minimum temperature value (according to datasheet)
    int maxTemp = 85;  // Replace with your maximum temperature value (accelerated to datasheet)
    if (temperatureC >= minTemp && temperatureC <= maxTemp)
    {
      saveCalibrationValues();
      Serial.println("Calibration complete and saved to EEPROM.");
    }
    else
    {
      Serial.println("Calibration complete but temperature out of range. Calibration not saved.");
    }
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
  readGyroData();
  processIMUData();
  calibrateMPU6050();
  // Check if the temperature is within the acceptable range
  int minTemp = -40; // Replace with your minimum temperature value
  int maxTemp = 85;  // Replace with your maximum temperature value
  if (temperatureC >= minTemp && temperatureC <= maxTemp)
  {
    Serial.println("Temperature data stored without EEPROM.");
  }
  else
  {
    Serial.println("Calibration complete but temperature out of range.");
  }
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
  const float gyroCoefficient = 0.3 / 65.5;                       // Coefficient for gyro input (deg/sec)
  const float gyroAngleCoefficient = 0.0000611;                   // Coefficient for gyro angle calculation
  const float radToDeg = 57.296;                                  // Radians to degrees conversion factor
  const float yawCoefficient = gyroAngleCoefficient * (PI / 180); // Coefficient for yaw calculations
  const float levelCorrectionFactor = 0.9996;                     // Factor for correcting drift with accelerometer

  // Gyro PID inputs
  gyro_roll_input = (gyro_roll_input * 0.7) + (gyro_roll * gyroCoefficient);
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (gyro_pitch * gyroCoefficient);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (gyro_yaw * gyroCoefficient);

  // Gyro angle calculations
  angle_pitch += gyro_pitch * gyroAngleCoefficient;
  angle_roll += gyro_roll * gyroAngleCoefficient;

  // // Place the accelgyro-6050 spirit level and note the values in the following two lines for calibration.
  // No need since we are already subtracted our offset.
  // angle_pitch_acc -= accYOffset; // Accelerometer calibration value for pitch.
  // angle_roll_acc -= accXOffset;  // Accelerometer calibration value for roll.

  // // Adjust pitch and roll angles based on yaw
  // //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * gyroAngleCoefficient); // If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * gyroAngleCoefficient); // If the IMU has yawed transfer the pitch angle to the roll angel.

  // Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

  // Then use acc_x for pitch and acc_y for roll
  if (abs(acc_y) < acc_total_vector)
  {                                                                     // Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * radToDeg; // Use acc_x for pitch
  }
  if (abs(acc_x) < acc_total_vector)
  {                                                                    // Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * radToDeg; // Use acc_y for roll
  }

  // Drift correction with accelerometer data
  angle_pitch = angle_pitch * levelCorrectionFactor + angle_pitch_acc * (1 - levelCorrectionFactor);
  angle_roll = angle_roll * levelCorrectionFactor + angle_roll_acc * (1 - levelCorrectionFactor);

  // Angle corrections for pitch and roll
  pitch_level_adjust = angle_pitch * 15;
  roll_level_adjust = angle_roll * 15;

  if (!auto_level)
  { // Disable level adjustments if auto-level is off
    pitch_level_adjust = 0;
    roll_level_adjust = 0;
  }
}

void Flight_Controller::motorControls()
{
  int local_channel_1, local_channel_2, local_channel_3, local_channel_4;
  noInterrupts();                             // Disable interrupts
  local_channel_3 = receiver_input_channel_1; // Throttle
  local_channel_4 = receiver_input_channel_4; // Yaw
  local_channel_1 = receiver_input_channel_2; // Roll
  local_channel_2 = receiver_input_channel_3; // Pitch
  interrupts();                               // Re-enable interrupt

  // start = 0 (off), start = 1 (ready to start), start = 2 (running)
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
      // No need to set start = 2 here as it's done in startInitializationSequence
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
  if (!(local_channel_3 < 1065 && local_channel_4 < 1050) && !(start == 1 && local_channel_3 < 1550 && local_channel_4 > 1450) && !(start == 2 && local_channel_3 <= 1064 && local_channel_4 > 1976))
  {
    isDebounceConditionMet = false;
  }

  pid_roll_setpoint = calculatePIDSetpoint(local_channel_1, roll_level_adjust);
  pid_pitch_setpoint = calculatePIDSetpoint(local_channel_2, pitch_level_adjust);
  pid_yaw_setpoint = calculatePIDSetpointForYaw(local_channel_3, local_channel_4);

  calculate_pid(); // Calculate PID with the new setpoints
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
  pid_i_mem_roll = pid_last_roll_d_error = 0;
  pid_i_mem_pitch = pid_last_pitch_d_error = 0;
  pid_i_mem_yaw = pid_last_yaw_d_error = 0;
}

float Flight_Controller::calculatePIDSetpoint(int channel, float level_adjust)
{
  float setpoint = 0;
  if (channel > 1508)
    setpoint = (channel - 1508) / 3.0;
  else if (channel < 1492)
    setpoint = (channel - 1492) / 3.0;
  setpoint -= level_adjust;
  return setpoint;
}

float Flight_Controller::calculatePIDSetpointForYaw(int channel_3, int channel_4)
{
  if (channel_3 <= 1050)
    return 0; // Do not yaw when turning off the motors
  if (channel_4 > 1508)
    return (channel_4 - 1508) / 3.0;
  if (channel_4 < 1492)
    return (channel_4 - 1492) / 3.0;
  return 0;
}

/**
 * @brief Calculates the PID outputs for roll, pitch, and yaw stabilization.
 *
 * This function computes the Proportional-Integral-Derivative (PID) controller outputs for each axis of the drone (roll, pitch, and yaw).
 * It takes the difference between the desired set-points and the actual gyro inputs to calculate the error. This error is then used to
 * calculate the PID output, which will be used to adjust the motors and move the drone towards the desired orientation.
 *
 * The function executes the following steps for each axis:
 * - Calculate the current error by subtracting the PID setpoint from the gyro input.
 * - Integrate the error over time, scaling by the integral gain, and limit this value to prevent integral windup.
 * - Calculate the derivative of the error, scaling by the derivative gain.
 * - Combine the proportional, integral, and derivative terms to get the total PID output, limiting this output to prevent overshooting.
 *
 * The PID outputs are constrained to a maximum value to ensure that the motor speeds remain within controllable limits.
 *
 * @note The constants for PID gains (pid_p_gain_roll, pid_i_gain_roll, pid_d_gain_roll, etc.) and maximums (pid_max_roll, pid_max_pitch, pid_max_yaw)
 *       need to be tuned to match the characteristics of the drone for stable flight.
 *
 * @param None
 * @return None
 */

void Flight_Controller::calculate_pid()
{
  pid_output_roll = calculate_pid_component(gyro_roll_input, pid_roll_setpoint, pid_i_mem_roll, pid_last_roll_d_error, pid_p_gain_roll, pid_i_gain_roll, pid_d_gain_roll, pid_max_roll);
  pid_output_pitch = calculate_pid_component(gyro_pitch_input, pid_pitch_setpoint, pid_i_mem_pitch, pid_last_pitch_d_error, pid_p_gain_pitch, pid_i_gain_pitch, pid_d_gain_pitch, pid_max_pitch);
  pid_output_yaw = calculate_pid_component(gyro_yaw_input, pid_yaw_setpoint, pid_i_mem_yaw, pid_last_yaw_d_error, pid_p_gain_yaw, pid_i_gain_yaw, pid_d_gain_yaw, pid_max_yaw);
}

float Flight_Controller::calculate_pid_component(float input, float setpoint, float &i_mem, float &last_d_error, float p_gain, float i_gain, float d_gain, float max_output)
{
  float error = input - setpoint;
  i_mem += i_gain * error;
  i_mem = constrain(i_mem, -max_output, max_output);

  float output = p_gain * error + i_mem + d_gain * (error - last_d_error);
  last_d_error = error;

  return constrain(output, -max_output, max_output); // make sure we don't go past our max.
}

void Flight_Controller::readGyroData()
{
  // Read gyro and accelerometer data here
  accelgyro.getMotion6(&acc_y, &acc_x, &acc_z, &gyro_roll, &gyro_pitch, &gyro_yaw);
}

void Flight_Controller::calibrateMPU6050()
{

  long buff_ax = 0, buff_ay = 0,
       buff_az = 0, buff_gx = 0,
       buff_gy = 0, buff_gz = 0;

  // collect a couple samples
  const int num_samples = 3000;
  for (int i = 0; i < num_samples; i++)
  {
    readGyroData();
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
  accZOffset = (buff_az / num_samples) - 16384; // considering 1g offset
  gyroXOffset = buff_gx / num_samples;
  gyroYOffset = buff_gy / num_samples;
  gyroZOffset = buff_gz / num_samples;
}

int16_t Flight_Controller::applyDeadzone(int16_t value, int16_t deadzone)
{
  if (abs(value) < deadzone)
  {
    return 0;
  }
  return value > 0 ? value - deadzone : value + deadzone;
}

void Flight_Controller::processIMUData()
{
  /*
    whenever we retrieve gyro and accelerometer data,
    we'll want to subtract these offsets from the raw values.
  */
  gyro_roll -= gyroXOffset;
  gyro_pitch -= gyroYOffset;
  gyro_yaw -= gyroZOffset;

  acc_x -= accXOffset;
  acc_y -= accYOffset;
  acc_z -= accZOffset;

  // Apply the deadzone
  acc_x = applyDeadzone(acc_x, 100);
  acc_y = applyDeadzone(acc_y, 100);
  acc_z = applyDeadzone(acc_z, 100);

  gyro_roll = applyDeadzone(gyro_roll, 1);
  gyro_pitch = applyDeadzone(gyro_pitch, 1);
  gyro_yaw = applyDeadzone(gyro_yaw, 1);

  gyro_roll = -gyro_roll; // Invert roll
  gyro_yaw = -gyro_yaw;   // Invert yaw

  // Invert accel x and y
  acc_x = -acc_x;
  acc_y = -acc_y;
}

void Flight_Controller::print()
{
  // // We don't use this data for our calculations
  // ax_mps2 = ((float)acc_x / 16384.0) * 9.81;
  // ay_mps2 = ((float)acc_y / 16384.0) * 9.81;
  // az_mps2 = ((float)acc_z / 16384.0) * 9.81;

  // Serial.print("--------------------");
  // Serial.println();
  // Serial.printf("Raw Gyro Pitch: %d \n", gyro_pitch);
  // Serial.printf("Raw Gyro Roll: %d \n", gyro_roll);
  // Serial.printf("Raw Gyro Yaw: %d \n", gyro_yaw);

  // Serial.print("--------------------");
  // Serial.println();
  // Serial.printf("Raw Acc X: %d \n ", acc_x);
  // Serial.printf("Raw Acc Y: %d \n ", acc_y);
  // Serial.printf("Raw Acc Z: %d \n ", acc_z);

  // Serial.print("--------------------");
  // Serial.println();
  // Serial.printf("Acc X (m/s^2): %.2f \n", ax_mps2);
  // Serial.printf("Acc Y (m/s^2): %.2f \n", ay_mps2);
  // Serial.printf("Acc Z (m/s^2): %.2f \n", az_mps2);

  // Serial.print("--------------------");
  // Serial.println();
  // Serial.printf("Angle Pitch: %.2f \n", angle_pitch);
  // Serial.printf("Angle Roll: %.2f \n", angle_roll);

  // Serial.print("--------------------");
  // Serial.println();
  // Serial.printf("angle_pitch_acc: %.2f \n", angle_pitch_acc);
  // Serial.printf("angle_roll_acc: %.2f \n", angle_roll_acc);

  // Serial.print("--------------------");
  // Serial.println();
  // Serial.printf("Pitch Adjust: %.2f \n", pitch_level_adjust);
  // Serial.printf("Roll Adjust: %.2f \n", roll_level_adjust);

  // Serial.print("--------------------");
  // Serial.println();
  // Serial.printf("gyro_roll_input: %.2f \n", gyro_roll_input);
  // Serial.printf("gyro_pitch_input: %.2f \n", gyro_pitch_input);
  // Serial.printf("gyro_yaw_input: %.2f \n", gyro_yaw_input);

  // Serial.print("--------------------");
  // Serial.println();
  // Serial.printf("pid_output_roll: %.2f \n", pid_output_roll);
  // Serial.printf("pid_output_pitch: %.2f \n", pid_output_pitch);
  // Serial.printf("pid_output_yaw: %.2f \n", pid_output_yaw);

  // Serial.print("--------------------");
  // Serial.println();
  // Serial.printf("pid_last_roll_d_error: %.2f \n", pid_last_roll_d_error);
  // Serial.printf("pid_last_pitch_d_error: %.2f \n", pid_last_pitch_d_error);
  // Serial.printf("pid_last_yaw_d_error: %.2f \n", pid_last_yaw_d_error);

  // // PID Values
  // Serial.print("--------------------");
  // Serial.println();
  // Serial.printf("pid_p_gain_roll: %.2f \n", pid_p_gain_roll);
  // Serial.printf("pid_i_gain_roll: %.2f \n", pid_i_gain_roll);
  // Serial.printf("pid_d_gain_roll: %.2f \n", pid_d_gain_roll);
  // Serial.println();
  // Serial.printf("pid_p_gain_pitch: %.2f \n", pid_p_gain_pitch);
  // Serial.printf("pid_i_gain_pitch: %.2f \n", pid_i_gain_pitch);
  // Serial.printf("pid_d_gain_pitch: %.2f \n", pid_d_gain_pitch);
  // Serial.println();
  // Serial.printf("pid_p_gain_yaw: %.2f \n", pid_p_gain_yaw);
  // Serial.printf("pid_i_gain_yaw: %.2f \n", pid_i_gain_yaw);
  // Serial.printf("pid_d_gain_yaw: %.2f \n", pid_d_gain_yaw);

  // Serial.println();
  // // Serial.printf("Start: %d \n", start);
  // Serial.printf("Throttle: %d \n", receiver_input_channel_1);
  // Serial.printf("Yaw:  %d \n", receiver_input_channel_4);
  // Serial.printf("Roll:  %d \n", receiver_input_channel_2);
  // Serial.printf("Pitch:  %d \n", receiver_input_channel_3);

  Serial.printf("ESC_1:  %d \n", esc_1);
  Serial.printf("ESC_2: %d \n ", esc_2);
  Serial.printf("ESC_3: %d \n ", esc_3);
  Serial.printf("ESC_4: %d \n ", esc_4);
}
