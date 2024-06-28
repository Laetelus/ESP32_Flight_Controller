#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Kalman.h"

Kalman kalmanRoll;
Kalman kalmanPitch;

float angle_pitch = 0.0, angle_roll = 0.0;
float gyro_pitch_input, gyro_roll_input, gyro_yaw_input;

int16_t ax_offset, ay_offset, az_offset;
int16_t gx_offset, gy_offset, gz_offset;
  int16_t raw_ax = 0, raw_ay = 0, raw_az = 0, raw_gx = 0, raw_gy = 0, raw_gz = 0;
  int16_t gyroXOffset, gyroYOffset, gyroZOffset, accXOffset, accYOffset, accZOffset;

static unsigned long loop_timer; 
const float alpha = 0.98; 

void initializeIMU();
void calibrateIMU();
void processIMUData(bool applyOffsets, bool applyFiltering);
void timer(); 

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock to 400kHz
  pinMode(2,OUTPUT);

  // Initialize and configure IMU
  initializeIMU();

  // Calibrate IMU
  calibrateIMU();

  Serial.println("MPU6050 initialized and calibrated.");
  loop_timer = millis();
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - loop_timer >= 4) { // Ensure loop runs at 250Hz
    loop_timer = currentTime;

    processIMUData(true, true);

    // Output the filtered angles for plotting and monitoring
    // Use Serial.println for monitor and plotter compatibility.
    #ifdef DEBUG
    Serial.print("Pitch: ");
    Serial.print(angle_pitch);
    Serial.print(", Roll: ");
    Serial.println(angle_roll);
    #else
    // This format is suitable for the Serial Plotter.
    Serial.print(angle_pitch);
    Serial.print(",");
    Serial.println(angle_roll);
    #endif
  }
}


void initializeIMU() {
  // Wake up MPU6050 and set clock source
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Power management register
  Wire.write(0x00); // Set clock source to internal 8 MHz oscillator
  Wire.endTransmission();

  // auto readRegister = [](uint8_t reg) -> uint8_t {
  //   Wire.beginTransmission(0x68);
  //   Wire.write(reg);
  //   Wire.endTransmission(false);
  //   Wire.requestFrom(0x68, 1);
  //   return Wire.available() ? Wire.read() : 0xFF; // Return 0xFF if no data available
  // };

  // // Check if power management was set correctly
  // if (readRegister(0x6B) != 0x00) {
  //   Serial.println("Failed to initialize clock source.");
  // }

  // // Set DLPF to reduce high-frequency noise
  // Wire.beginTransmission(0x68);
  // Wire.write(0x1A);
  // Wire.write(0x05);
  // Wire.endTransmission();

  // // Check if DLPF was set correctly
  // if (readRegister(0x1A) != 0x05) {
  //   Serial.println("Failed to set DLPF.");
  // }

  // // Set accelerometer sensitivity to ±8g
  // Wire.beginTransmission(0x68);
  // Wire.write(0x1C);
  // Wire.write(0x10);
  // Wire.endTransmission();

  // // Check if accelerometer sensitivity was set correctly
  // if (readRegister(0x1C) != 0x10) {
  //   Serial.println("Failed to set accelerometer sensitivity to ±8g.");
  // } else {
  //   Serial.println("Accelerometer sensitivity range set to ±8g.");
  // }

  // // Set gyroscope sensitivity to ±500 deg/s
  // Wire.beginTransmission(0x68);
  // Wire.write(0x1B);
  // Wire.write(0x08);
  // Wire.endTransmission();

  // // Check if gyroscope sensitivity was set correctly
  // if (readRegister(0x1B) != 0x08) {
  //   Serial.println("Failed to set gyroscope sensitivity to ±500 deg/s.");
  // } else {
  //   Serial.println("Gyroscope sensitivity range set to ±500 deg/s.");
  // }

  Serial.println("Initialization and Configuration Complete.");
}

void calibrateIMU() {
  const int calibrationSamples = 5000;
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  Serial.println("Calibrating IMU...");
  for (int i = 0; i < calibrationSamples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    processIMUData(false, false); // Collect raw data for calibration

    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;

    delay(4);
  }

  ax_offset = ax_sum / calibrationSamples;
  ay_offset = ay_sum / calibrationSamples;
  az_offset = az_sum / calibrationSamples - 4096; // Assuming 4096 is correct for your sensor range

  gx_offset = gx_sum / calibrationSamples;
  gy_offset = gy_sum / calibrationSamples;
  gz_offset = gz_sum / calibrationSamples;

  Serial.println("IMU Calibration Complete.");
}

  void processIMUData(bool applyOffsets, bool applyFiltering)
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
    float gyroRateX = (float)raw_gx / 65.5; // Gyro sensitivity for ±500 dps
    float gyroRateY = (float)raw_gy / 65.5;
    float gyroRateZ = (float)raw_gz / 65.5;

    // Convert accelerometer readings from raw to g's
    // Accel sensitivity for ±8g
    float ax_g = (float)raw_ax / 4096.0 + 0.01; // Do not forget to Modify each value
    float ay_g = (float)raw_ay / 4096.0 + 0.01;
    float az_g = (float)raw_az / 4096.0 + 0.001;
    
    // ax_g = (float)raw_ax / 4096.0 + 0.05; //Do not forget to Modify each value
    // ay_g = (float)raw_ay / 4096.0 + 0.01;
    // az_g = (float)raw_az / 4096.0 + 0.001;

    // accRoll = atan(ay_g / sqrt(ax_g * ax_g + az_g * az_g)) * 1 / (3.142 / 180);
    // accPitch = -atan(ax_g / sqrt(ay_g * ay_g + az_g * az_g)) * 1 / (3.142 / 180);
    
    float accRoll = atan(ay_g / sqrt(ax_g * ax_g + az_g * az_g)) * RAD_TO_DEG;
    float accPitch = atan2(-ax_g, az_g) * RAD_TO_DEG;

    // accRoll = atan2(ay_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180 / PI; // Roll calculation
    // accPitch = -atan2(ax_g, az_g) * 180 / PI;                           // Pitch calculation

    // accRoll = atan(ay_g / sqrt(-ax_g * ax_g + az_g * az_g)) * RAD_TO_DEG;
    // accPitch = atan2(ax_g, az_g) * RAD_TO_DEG;

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
      // 98% from integrated gyro angles, 2% from accelerometer
      angle_roll = 0.98 * (angle_roll + gyroRateX * dt) + 0.02 * accRoll;
      angle_pitch = 0.98 * (angle_pitch + gyroRateY * dt) + 0.02 * accPitch;
    }

    // Update gyro inputs for PID calculations
    gyro_roll_input = gyroRateX;
    gyro_pitch_input = gyroRateY;
    gyro_yaw_input = gyroRateZ;
  }

