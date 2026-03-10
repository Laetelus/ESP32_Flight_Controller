#include "IMU.h"


void IMU::initializeI2CBus()
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
  
  //----pointing temp sensor-----------------
  Wire.beginTransmission(0x68);                        //Start communication with the MPU-6050.
  Wire.write(0x41);      //pointing Temp_Out_High Reg                                           //Set the register bits as 00000000 to activate the gyro.
  Wire.endTransmission();

  Wire.requestFrom(0x68, 2, true); // Request 2 bytes from TEMP_OUT_H and TEMP_OUT_L
  byte tempH = Wire.read();
  byte tempL = Wire.read();
  int16_t tempRaw = (tempH << 8) | tempL;  

  //Convert raw value to temperature in °F (my preference)
  float temperatureC = (tempRaw / 340.0) + 36.53;
  temperatureF_ = (temperatureC * 9.0 / 5.0) + 32.0;   

}

void IMU::processIMUData() {

  // Read raw IMU data from MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Start at the accelerometer data register
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14); // Request 14 bytes: 6 acc, 2 temp, 6 gyro

  // Read raw accelerometer data
  raw_.ax = Wire.read() << 8 | Wire.read();
  raw_.ay = Wire.read() << 8 | Wire.read();
  raw_.az = Wire.read() << 8 | Wire.read();

  // Read temperature data (not used in this implementation)
  Wire.read() << 8 | Wire.read();

  // Read raw gyroscope data
  raw_.gx = Wire.read() << 8 | Wire.read();
  raw_.gy = Wire.read() << 8 | Wire.read();
  raw_.gz = Wire.read() << 8 | Wire.read();

}

void IMU::scaleIMU()
{

  // --- subtract OFFSETS IN RAW UNITS ---
  int16_t gx = raw_.gx - ofst_.gyroX;
  int16_t gy = raw_.gy - ofst_.gyroY;
  int16_t gz = raw_.gz - ofst_.gyroZ;

  int16_t ax = raw_.ax - ofst_.accX;
  int16_t ay = raw_.ay - ofst_.accY;
  int16_t az = raw_.az - ofst_.accZ;

  // --- Gyros scaled to deg/s ---
  scaled_.gx_dps = gx * (500.0f / 32768.0f);
  scaled_.gy_dps = gy * (500.0f / 32768.0f);
  scaled_.gz_dps = gz * (500.0f / 32768.0f);

  // --- Accel scaled to g ---
  scaled_.ax_g = ax * (8.0f / 32768.0f);
  scaled_.ay_g = ay * (8.0f / 32768.0f);
  scaled_.az_g = az * (8.0f / 32768.0f);

//   Serial.println(ofst_.gyroX); 
//   Serial.println(ofst_.gyroY);
//   Serial.println(ofst_.gyroZ);

//   Serial.println(ofst_.accX);
//   Serial.println(ofst_.accY);
//   Serial.println(ofst_.accZ);

}

void IMU::calcAccelAngle()
{

  // Use atan2(ay, az) for roll in a flight controller.
  // The sqrt version is an approximation that only works when pitch is small.
  accA_.Roll = atan2(scaled_.ay_g, scaled_.az_g) * RAD_TO_DEG;

  accA_.Pitch = atan2(-scaled_.ax_g, sqrt(scaled_.ay_g * scaled_.ay_g 
                            + scaled_.az_g * scaled_.az_g)) * RAD_TO_DEG;

  Serial.println(accA_.Roll); 
  Serial.println(accA_.Pitch);

}