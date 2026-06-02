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
  Wire.beginTransmission(0x68); //Start communication with the MPU-6050.
  Wire.write(0x41); //pointing Temp_Out_High Reg                                        
  Wire.endTransmission();

  Wire.requestFrom(0x68, 2, true); // Request 2 bytes from TEMP_OUT_H and TEMP_OUT_L
  byte tempH = Wire.read();
  byte tempL = Wire.read();
  int16_t tempRaw = (tempH << 8) | tempL;  

  //Convert raw value to temperature in °F (my preference)
  float temperatureC = (tempRaw / 340.0) + 36.53;
  temperatureF_ = (temperatureC * 9.0 / 5.0) + 32.0;   

}

void IMU::readRawIMUData() {

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
  const float raw_gx_dps = gx * (500.0f / 32768.0f);
  const float raw_gy_dps = gy * (500.0f / 32768.0f);
  const float raw_gz_dps = gz * (500.0f / 32768.0f);

  //TODO: How do we test this visually? 

  // IIR low-pass filter: 70% previous + 30% new sample.
  // Attenuates high-frequency vibration noise before gyro rates reach
  // the PID D-term — without this, frame vibrations alias into D and
  // cause motor buzz / oscillation.
  lpf_gx_ = 0.7f * lpf_gx_ + 0.3f * raw_gx_dps;
  lpf_gy_ = 0.7f * lpf_gy_ + 0.3f * raw_gy_dps;
  lpf_gz_ = 0.7f * lpf_gz_ + 0.3f * raw_gz_dps;

  scaled_.gx_dps = lpf_gx_;
  scaled_.gy_dps = lpf_gy_;
  scaled_.gz_dps = lpf_gz_;

  // --- Accel scaled to g ---
  scaled_.ax_g = ax * (8.0f / 32768.0f);
  scaled_.ay_g = ay * (8.0f / 32768.0f);
  scaled_.az_g = az * (8.0f / 32768.0f);

}

AccelAngleData IMU::calcAccelAngle()
{

  // Use atan2(ay, az) for roll in a flight controller.
  // The sqrt version is an approximation that only works when pitch is small.
  accA_.Roll = atan2(scaled_.ay_g, scaled_.az_g) * RAD_TO_DEG;

  accA_.Pitch = atan2(-scaled_.ax_g, sqrt(scaled_.ay_g * scaled_.ay_g 
                            + scaled_.az_g * scaled_.az_g)) * RAD_TO_DEG;

  return accA_; 
}

void IMU::syncAttitudeToAccel()
{
  const AccelAngleData accel = calcAccelAngle();
  attitude_.roll_deg  = accel.Roll;
  attitude_.pitch_deg = accel.Pitch;
}

FilteredAttitude IMU::compFilter(const ScaledImuData &scaled)
{

  // alpha=0.98 → τ = dt/(1-alpha) = 0.004/0.02 = 0.2s convergence to accel.
  // The IIR gyro low-pass filter handles vibration rejection, so alpha doesn't
  // need to be as conservative as 0.9996 (which gives a 10s time constant).
  const float alpha = 0.98f;

  attitude_.roll_deg  += scaled.gx_dps * dt; 
  attitude_.pitch_deg += scaled.gy_dps * dt;

  // Yaw coupling correction: when the IMU yaws, the physical pitch/roll axes
  // rotate in space. Without this, a 90° yaw would gradually swap pitch and roll
  // in the integrated angles. sin(yaw_rad_this_dt) is the tiny cross-axis
  // transfer each frame — gz_dps * dt converts the yaw rate to radians for this step.
  const float yaw_rad = scaled.gz_dps * dt * DEG_TO_RAD;
  
  attitude_.pitch_deg -= attitude_.roll_deg  * sinf(yaw_rad);
  attitude_.roll_deg  += attitude_.pitch_deg * sinf(yaw_rad);

  AccelAngleData accA_ = calcAccelAngle();

  attitude_.roll_deg  = alpha * attitude_.roll_deg
                   + (1.0f - alpha) * accA_.Roll;

  attitude_.pitch_deg = alpha * attitude_.pitch_deg
                   + (1.0f - alpha) * accA_.Pitch;

  return attitude_;
}

