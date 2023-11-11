#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>
#include <EEPROM.h>
#include "Flight_Controller.h"

/*
  uncomment DEBUG_IMU to view IMU on webserver. DO NOT! upload both the flight controller
  software and the Webserver software if not testing IMU. 
*/ 
//#define DEBUG_IMU

//#define USE_EEPROM

// Flight controller interface 
void Flight_Controller::initialize()
{

  Serial.begin(115200);
  pinMode(2,OUTPUT); //LED status 
  #ifdef DEBUG_IMU
    initWiFi();
    initSPIFFS();
    Handle_Server(); 
  #endif

  // Allow allocation of all timers. Consistent and accurate PWM. 
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
     Fastwire::setup(400, true);
  #endif

  // initialize device. This also initialises the gyros and accelerometers 
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  // Verify gyroscope range
  uint8_t gyroRange = accelgyro.getFullScaleGyroRange();
  if (gyroRange == MPU6050_IMU::MPU6050_GYRO_FS_250) {
    Serial.println("Gyroscope range verified as ±250°/s");
  } else {
    Serial.println("Gyroscope range verification failed");
  }

  // Verify accelerometer range
  uint8_t accelRange = accelgyro.getFullScaleAccelRange();
  if (accelRange == MPU6050_IMU::MPU6050_ACCEL_FS_2) {
    Serial.println("Accelerometer range verified as ±2g");
  } else {
    Serial.println("Accelerometer range verification failed");
  }
  // verify connection
  #ifdef I2CDEV_IMPLEMENTATION
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "accelgyro6050 connection successful" : "accelgyro6050 connection failed");
  #endif   

  delay(100);  

  //Be careful with commenting and uncommenting this function. 
  //clearCalibrationData();

  // Warm-up time for the MPU6050 (LED will stay on for 3secs to indicate warm-up time)
  Serial.println("Warming up MPU6050...");
  digitalWrite(2, HIGH);
  delay(3000);
  digitalWrite(2, LOW);
  Serial.println("Warm-up complete.");

  // Read current temperature
  temp = accelgyro.getTemperature();
  temperatureC = float(temp) / 340.00 + 36.53;
  Serial.print("Current temperature: ");
  Serial.println(temperatureC);

  // Calibration with or without EEPROM (only store EEPROM when data is satisfactory)
  #ifdef USE_EEPROM
    EEPROM.begin(EEPROM_SIZE);
    if (!loadCalibrationValues()) {
      Serial.println("Calibration data not found in EEPROM. Calibrating...");
      // Blink LED to indicate calibration in progress
      for (int i = 0; i < 10; ++i) {
        digitalWrite(2, HIGH);
        delay(100);
        digitalWrite(2, LOW);
        delay(100);
      }
      calibrateMPU6050();
      // Check if the temperature is within the acceptable range
      int minTemp = -40; // Replace with your minimum temperature value (according to datasheet)
      int maxTemp = 85; // Replace with your maximum temperature value (accelerated to datasheet)
      if (temperatureC >= minTemp && temperatureC <= maxTemp) {
        saveCalibrationValues();
        Serial.println("Calibration complete and saved to EEPROM.");
      } else {
        Serial.println("Calibration complete but temperature out of range. Calibration not saved.");
      }
     // Turn off LED
    digitalWrite(2, LOW);
  } else {
    Serial.println("Calibration data loaded from EEPROM.");
  }
  #else
    Serial.println("Calibrating without EEPROM...");
    digitalWrite(2, HIGH);
    calibrateMPU6050();
    // Check if the temperature is within the acceptable range
    int minTemp = -40; // Replace with your minimum temperature value
    int maxTemp = 85;  // Replace with your maximum temperature value
    if (temperatureC >= minTemp && temperatureC <= maxTemp) {
      Serial.println("Temperature data stored without EEPROM.");
    } else {
      Serial.println("Calibration complete but temperature out of range.");
    }
    digitalWrite(2, LOW);
  #endif
  
  printStoredCalibrationValues(); 
 
  // attach esc pins
  esc1.attach(esc_pin1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // FR (Front Right)
  esc2.attach(esc_pin3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // BR (Back Right)
  esc3.attach(esc_pin2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // FL (Front Left)
  esc4.attach(esc_pin4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // BL (Back Left)

  // arm ecs
  esc1.writeMicroseconds(MIN_PULSE_LENGTH);
  esc2.writeMicroseconds(MIN_PULSE_LENGTH);
  esc3.writeMicroseconds(MIN_PULSE_LENGTH);
  esc4.writeMicroseconds(MIN_PULSE_LENGTH);

  //setup our inputs 
  pinMode(THROTTLE, INPUT);
  pinMode(YAW, INPUT);
  pinMode(PITCH, INPUT);
  pinMode(ROLL, INPUT);
}

void Flight_Controller::read_Controller() {
    // Read the raw channel values
    receiver_input_channel_3 = pulseIn(THROTTLE, HIGH, 25000); // Throttle
    receiver_input_channel_4 = pulseIn(YAW, HIGH, 25000);      // Yaw
    receiver_input_channel_1 = pulseIn(ROLL, HIGH, 25000);     // Roll
    receiver_input_channel_2 = pulseIn(PITCH, HIGH, 25000);    // Pitch
}

void Flight_Controller::level_flight() {
  // 65.5 = 1 deg/sec (check the datasheet of the accelgyro-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);    // Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3); // Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);       // Gyro pid input is deg/sec.

  // Gyro angle calculations
  // 0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611; // Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0000611;   // Calculate the traveled roll angle and add this to the angle_roll variable.

  // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066); //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066); //If the IMU has yawed transfer the pitch angle to the roll angel.

  // Accelerometer angle calculations
  acc_total_vector = sqrt((ax_mps2 * ax_mps2) + (ay_mps2 * ay_mps2) + (az_mps2 * az_mps2));

  if (abs(ax_mps2) < acc_total_vector)
  { // Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)ax_mps2 / acc_total_vector) * 57.296; // Calculate the pitch angle.
  }
  if (abs(ay_mps2) < acc_total_vector)
  {  // Prevent the asin function to produce a NaN
    angle_roll_acc = -asin((float)ay_mps2 / acc_total_vector) * -57.296; // Calculate the roll angle.
  }

  // // Place the accelgyro-6050 spirit level and note the values in the following two lines for calibration.
  //No need since we are already subtracting our offset. 
  // angle_pitch_acc -= 0.0; // Accelerometer calibration value for pitch.
  // angle_roll_acc -= 0.0;  // Accelerometer calibration value for roll.

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

void Flight_Controller::motorControls() {
  
  // For starting the motors: throttle low and yaw left (step 1).
  if (receiver_input_channel_3 < 1065 && receiver_input_channel_4 < 1050)
  {
    start = 1;
  }

  // When yaw stick is back in the center position start the motors (step 2).
  if (start == 1 && receiver_input_channel_3 < 1550 && receiver_input_channel_4 > 1450)
  {
    start = 2;

    angle_pitch = angle_pitch_acc; // Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;   // Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    
    gyro_angles_set = true;        // Set the IMU started flag.

    // Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }

  // Stopping the motors: throttle low and yaw right.
  if (start == 2 && receiver_input_channel_3 <= 1064 && receiver_input_channel_4 > 1976)
  {
    start = 0;
  }

  // The PID set point in degrees per second is determined by the roll receiver input.
  // In the case of dividing by 3 the max roll rate is approx 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;

  // We need a little dead band of 16us for better results.
  if (receiver_input_channel_1 > 1508)
  {
    pid_roll_setpoint = receiver_input_channel_1 - 1508;
  }
  else if (receiver_input_channel_1 < 1492)
  {
    pid_roll_setpoint = receiver_input_channel_1 - 1492;
  }

  pid_roll_setpoint -= roll_level_adjust; // Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;               // Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

  // The PID set point in degrees per second is determined by the pitch receiver input.
  // In the case of dividing by 3 the max pitch rate is approx 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;

  // We need a little dead band of 16us for better results.
  if (receiver_input_channel_2 > 1508)
  {
    pid_pitch_setpoint = receiver_input_channel_2 - 1508;
  }
  else if (receiver_input_channel_2 < 1492)
  {
    pid_pitch_setpoint = receiver_input_channel_2 - 1492;
  }

  pid_pitch_setpoint -= pitch_level_adjust; // Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                // Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  // The PID set point in degrees per second is determined by the yaw receiver input.
  // In the case of dividing by 3 the max yaw rate is approx 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;

  // We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050)
  { // Do not yaw when turning off the motors.
    if (receiver_input_channel_4 > 1508)
      pid_yaw_setpoint = (receiver_input_channel_4 - 1508) / 3.0;
    else if (receiver_input_channel_4 < 1492)
      pid_yaw_setpoint = (receiver_input_channel_4 - 1492) / 3.0;
  }
    calculate_pid(); // PID inputs are known. So we can calculate the pid output.
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
void Flight_Controller::calculate_pid() {
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


void Flight_Controller::readGyroData() {
    // Read gyro and accelerometer data here
    accelgyro.getMotion6(&acc_x, &acc_y, &acc_z, &gyro_roll, &gyro_pitch, &gyro_yaw);
}

void Flight_Controller::calibrateMPU6050() {

  long buff_ax = 0, buff_ay = 0, 
  buff_az = 0, buff_gx = 0, 
  buff_gy = 0, buff_gz = 0;

  //collect a couple samples 
  const int num_samples = 3000;
  for (int i = 0; i < num_samples; i++) {
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
  accXOffset  = buff_ax  / num_samples;
  accYOffset  = buff_ay  / num_samples;
  accZOffset  =(buff_az  / num_samples) - 16384; // considering 1g offset
  gyroXOffset = buff_gx / num_samples;
  gyroYOffset = buff_gy / num_samples;
  gyroZOffset = buff_gz / num_samples;

}

void Flight_Controller::saveCalibrationValues() {
  if (EEPROM.readLong(0) != 0x12345678) {
    EEPROM.writeLong(0, 0x12345678);  // Unique identifier indicating values have been saved
    EEPROM.writeLong(4, accXOffset);
    EEPROM.writeLong(8, accYOffset);
    EEPROM.writeLong(12, accZOffset);
    EEPROM.writeLong(16, gyroXOffset);
    EEPROM.writeLong(20, gyroYOffset);
    EEPROM.writeLong(24, gyroZOffset);
    EEPROM.writeFloat(28, temperatureC); 
    EEPROM.commit();
    Serial.println("Calibration values saved to EEPROM");
  } else {
    Serial.println("EEPROM already contains calibration values. Skipping save.");
  }
}

bool Flight_Controller::loadCalibrationValues() {
  if (EEPROM.readLong(0) != 0x12345678) {
    return false; // Calibration data not found or not valid
  }
  accXOffset  = EEPROM.readLong(4);
  accYOffset  = EEPROM.readLong(8);
  accZOffset  = EEPROM.readLong(12);
  gyroXOffset = EEPROM.readLong(16);
  gyroYOffset = EEPROM.readLong(20);
  gyroZOffset = EEPROM.readLong(24);
  temperatureC = EEPROM.readFloat(28);  
  return true;
}


//Only used if needed to get new values or writing did not go well. 
void Flight_Controller::clearCalibrationData() {
  EEPROM.begin(EEPROM_SIZE);
  // Set a specific value to indicate that the data is cleared or invalid
  long invalidValue = 0;
  
  EEPROM.writeLong(0, invalidValue);
  EEPROM.writeLong(4, invalidValue);
  EEPROM.writeLong(8, invalidValue);
  EEPROM.writeLong(12, invalidValue);
  EEPROM.writeLong(16, invalidValue);
  EEPROM.writeLong(20, invalidValue);
  EEPROM.writeLong(24, invalidValue); 
  EEPROM.writeFloat(28, invalidValue); 

  EEPROM.commit(); // Make sure to commit the changes to EEPROM
  Serial.println("Calibration data cleared.");
}

void Flight_Controller::printStoredCalibrationValues() {

  // Addresses where the calibration values are stored
  const int gyroXOffsetAddr = 4;
  const int gyroYOffsetAddr = 8;
  const int gyroZOffsetAddr = 12;
  const int accXOffsetAddr = 16;
  const int accYOffsetAddr = 20;
  const int accZOffsetAddr = 24;
  const int tempOffsetAddr = 28;  

  // Check for unique ID
  if (EEPROM.readLong(0) == 0x12345678) {
    // Read and print stored offset values
    int32_t storedGyroXOffset = EEPROM.readLong(gyroXOffsetAddr);
    int32_t storedGyroYOffset = EEPROM.readLong(gyroYOffsetAddr);
    int32_t storedGyroZOffset = EEPROM.readLong(gyroZOffsetAddr);
    int32_t storedAccXOffset = EEPROM.readLong(accXOffsetAddr);
    int32_t storedAccYOffset = EEPROM.readLong(accYOffsetAddr);
    int32_t storedAccZOffset = EEPROM.readLong(accZOffsetAddr);
    float storedTempOffset = EEPROM.readFloat(tempOffsetAddr);

    Serial.println("Stored calibration values in EEPROM:");
    Serial.print("Gyro X Offset: "); Serial.println(storedGyroXOffset);
    Serial.print("Gyro Y Offset: "); Serial.println(storedGyroYOffset);
    Serial.print("Gyro Z Offset: "); Serial.println(storedGyroZOffset);
    Serial.print("Acc X Offset: "); Serial.println(storedAccXOffset);
    Serial.print("Acc Y Offset: "); Serial.println(storedAccYOffset);
    Serial.print("Acc Z Offset: "); Serial.println(storedAccZOffset);
    Serial.print("Temperature: "); Serial.println(tempOffsetAddr);
  } else {
    Serial.println("No valid data found in EEPROM");
  }
}



int16_t Flight_Controller::applyDeadzone(int16_t value, int16_t deadzone) {
  if (abs(value) < deadzone) {
    return 0;
  }
  return value > 0 ? value - deadzone : value + deadzone;
}

void Flight_Controller::processIMUData() {
  /*
    whenever we retrieve gyro and accelerometer data, 
    we'll want to subtract these offsets from the raw values. 
  */
  gyro_roll  -= gyroXOffset;
  gyro_pitch -= gyroYOffset;
  gyro_yaw   -= gyroZOffset;
  acc_x -= accXOffset;
  acc_y -= accYOffset;
  acc_z -= accZOffset;

  // Apply the deadzone
  acc_x = applyDeadzone(acc_x, 100);
  acc_y = applyDeadzone(acc_y, 100);
  acc_z = applyDeadzone(acc_z, 100);
    
  gyro_roll  = applyDeadzone(gyro_roll, 1);
  gyro_pitch = applyDeadzone(gyro_pitch, 1);
  gyro_yaw   = applyDeadzone(gyro_yaw , 1);

  gyro_roll = -gyro_roll; // Invert the roll
  gyro_yaw = -gyro_yaw;  // Invert the roll

  /*All three axes of the accelerometer (X, Y, and Z) measure acceleration 
    in units of g, and thus all three need to be converted to m/s² to have 
    consistent units across all axes. 
  */
  ax_mps2 = -((float)acc_x / 16384.0) * 9.81;
  ay_mps2 = -((float)acc_y / 16384.0) * 9.81;
  az_mps2 = ((float)acc_z / 16384.0) * 9.81;

  #ifdef DEBUG_IMU
    Send_Event();
  #endif // DEBUG_IMU
}


void Flight_Controller::mix_motors() {
    throttle = receiver_input_channel_3; // We need the throttle signal as a base signal.
    if (start == 2) {                   // The motors are started.
        if (throttle > 1800)
            throttle = 1800; // We need some room to keep full control at full throttle.

        // Mixing algorithm for appropriate motors
        esc_1 = constrain(map(throttle - pid_output_pitch + pid_output_roll - pid_output_yaw, 1000, 2000, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH), MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // FR (Front Right)
        esc_2 = constrain(map(throttle + pid_output_pitch + pid_output_roll + pid_output_yaw, 1000, 2000, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH), MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // BR (Back Right)
        esc_3 = constrain(map(throttle - pid_output_pitch - pid_output_roll + pid_output_yaw, 1000, 2000, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH), MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // FL (Front Left)
        esc_4 = constrain(map(throttle + pid_output_pitch - pid_output_roll - pid_output_yaw, 1000, 2000, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH), MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // BL (Back Left)

        if (esc_1 < 1100)
            esc_1 = 1100; // Keep the motors running.
        if (esc_2 < 1100)
            esc_2 = 1100; // Keep the motors running.
        if (esc_3 < 1100)
            esc_3 = 1100; // Keep the motors running.
        if (esc_4 < 1100)
            esc_4 = 1100; // Keep the motors running.

        if (esc_1 > 2000)
            esc_1 = 2000; // Limit the esc-1 pulse to 2000us.
        if (esc_2 > 2000)
            esc_2 = 2000; // Limit the esc-2 pulse to 2000us.
        if (esc_3 > 2000)
            esc_3 = 2000; // Limit the esc-3 pulse to 2000us.
        if (esc_4 > 2000)
            esc_4 = 2000; // Limit the esc-4 pulse to 2000us.
    } else {
        esc_1 = 1000; // If start is not 2, keep a 1000us pulse for esc-1.
        esc_2 = 1000; // If start is not 2, keep a 1000us pulse for esc-2.
        esc_3 = 1000; // If start is not 2, keep a 1000us pulse for esc-3.
        esc_4 = 1000; // If start is not 2, keep a 1000us pulse for esc-4.
    }
}

void Flight_Controller::write_motors(){
    readGyroData();
    processIMUData();

    esc1.writeMicroseconds(esc_1);
    esc2.writeMicroseconds(esc_2);
    esc3.writeMicroseconds(esc_3);
    esc4.writeMicroseconds(esc_4);
}

void Flight_Controller::print_gyro_data() {

  Serial.print("\nRaw Gyro Pitch: "); 
  Serial.println(gyro_pitch);
  
  Serial.print("Raw Gyro Roll: "); 
  Serial.println(gyro_roll);
  
  Serial.print("Raw Gyro Yaw: "); 
  Serial.println(gyro_yaw);

  Serial.print("--------------------"); 
  Serial.print("\nAngle Pitch: "); 
  Serial.println(angle_pitch);

  Serial.print("Angle Roll: "); 
  Serial.println(angle_roll);

  Serial.print("--------------------");   
  Serial.print("\nangle_pitch_acc: ");
  Serial.println(angle_pitch_acc);

  Serial.print("angle_roll_acc: ");
  Serial.println(angle_roll_acc);

  Serial.print("--------------------"); 
  Serial.print("\nPitch Adjust: "); 
  Serial.println(pitch_level_adjust);

  Serial.print("Roll Adjust: "); 
  Serial.println(roll_level_adjust);

  Serial.print("--------------------"); 
  Serial.print("\nAcc X: "); 
  Serial.println(ax_mps2);

  Serial.print("Acc Y: "); 
  Serial.println(ay_mps2);

  Serial.print("Acc Z (m/s^2): "); 
  Serial.println(az_mps2);
  Serial.print("--------------------"); 
}