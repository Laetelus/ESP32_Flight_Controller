#include <Wire.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"

// Function declarations
void processIMUData(bool applyOffsets);
void level_flight();
void timer();
void initializeI2CBus();

// MPU6050 variables
float acc_x, acc_y, acc_z;
float gyro_roll, gyro_pitch, gyro_yaw;
float gyro_roll_input, gyro_pitch_input, gyro_yaw_input;
float accXOffset = 0, accYOffset = 0, accZOffset = 0;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
float angle_pitch, angle_roll;
float angle_pitch_acc, angle_roll_acc;
float acc_total_vector;
bool auto_level = true;
unsigned long loop_timer;
  int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;

void setup()
{
  Serial.begin(115200);
  initializeI2CBus();
  pinMode(2, OUTPUT); // LED status
  Serial.println("MPU6050 initialization complete.");
  loop_timer = esp_timer_get_time();

  // Print headers for Serial Plotter
  Serial.println("Time, Gyro Roll, Gyro Pitch");
}

void loop()
{
  // Process IMU data
  processIMUData(true);

  // Apply complementary filter for level flight
  level_flight();




  // Print data for plotting
  Serial.print(", ");
  Serial.print(gyro_roll);
  Serial.print(", ");
  Serial.println(gyro_pitch);

  // Maintain the loop timing at 250Hz
  timer();
}

void processIMUData(bool applyOffsets)
{


  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14);

  raw_ay = Wire.read() << 8 | Wire.read();
  raw_ax = Wire.read() << 8 | Wire.read();
  raw_az = Wire.read() << 8 | Wire.read();
  Wire.read();
  Wire.read();
  raw_gx = Wire.read() << 8 | Wire.read();
  raw_gy = Wire.read() << 8 | Wire.read();
  raw_gz = Wire.read() << 8 | Wire.read();

  float temp_acc_x = (float)raw_ax / 4096;
  float temp_acc_y = (float)raw_ay / 4096;
  float temp_acc_z = (float)raw_az / 4096;

  float temp_gyro_roll = (float)raw_gx / 65.5;
  float temp_gyro_pitch = (float)raw_gy / 65.5;
  float temp_gyro_yaw = (float)raw_gz / 65.5;

  if (applyOffsets)
  {
    acc_x = temp_acc_x - accXOffset;
    acc_y = temp_acc_y - accYOffset;
    acc_z = temp_acc_z - accZOffset;
    gyro_roll = temp_gyro_roll - gyroXOffset;
    gyro_pitch = temp_gyro_pitch - gyroYOffset;
    gyro_yaw = temp_gyro_yaw - gyroZOffset;
  }
  else
  {
    acc_x = temp_acc_x;
    acc_y = temp_acc_y;
    acc_z = temp_acc_z;
    gyro_roll = temp_gyro_roll;
    gyro_pitch = temp_gyro_pitch;
    gyro_yaw = temp_gyro_yaw;
  }

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

  gyro_roll_input = gyro_roll;
  gyro_pitch_input = gyro_pitch;
  gyro_yaw_input = gyro_yaw;
}

void level_flight()
{
  gyro_roll_input = (gyro_roll_input * 0.7) + (gyro_roll * 0.3);
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (gyro_pitch * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (gyro_yaw * 0.3);

  angle_pitch += gyro_pitch * 0.0000611;
  angle_roll += gyro_roll * 0.0000611;
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);

  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

  if (abs(acc_y) < acc_total_vector)
  {
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;
  }
  if (abs(acc_x) < acc_total_vector)
  {
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;
  }

  angle_pitch_acc -= accXOffset;
  angle_roll_acc -= accYOffset;

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
}

void timer()
{
  unsigned long time_taken = esp_timer_get_time() - loop_timer;
  if (time_taken > 4000)
  {
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
  }
  while (esp_timer_get_time() - loop_timer < 4000)
    ;
  loop_timer = esp_timer_get_time();
}

void initializeI2CBus()
{
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}
