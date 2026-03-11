#include <Arduino.h>
#include <EEPROM.h>
#include "Calibration.h"
#include "Flight_Controller.h"
#include "IMU.h"

void Calibration::performCalibration()
{

  const RawImuData& raw = imu.getRawData();
  const AccelAngleData& ang = imu.getAccelAngles();
  const ScaledImuData& scale = imu.getScaledData();
  ImuOffsets ofst;

  // Run for 2 seconds, store as the sample count 
  unsigned long cal_time = 2000; 
  long buff_ax = 0, buff_ay = 0,
       buff_az = 0, buff_gx = 0,
       buff_gy = 0, buff_gz = 0;

  int sampleCount = 0;

#ifdef USE_EEPROM

  EEPROM.begin(EEPROM_SIZE);

  if (!loadCalibrationValues())
  {

    Serial.println("Calibration data not found in EEPROM. Calibrating...");
    // Blink LED to indicate calibration is in progress
    digitalWrite(2, HIGH); // Turn on LED to indicate start of calibration

    unsigned long startTime = millis();
    while (millis() - startTime < cal_time)
    {
      imu.readRawIMUData(); // Collect raw data for calibration period
      buff_ax += raw.ax;
      buff_ax += raw.ay;
      buff_ax += raw.az;

      buff_gx += raw.gx;
      buff_gy += raw.gy;
      buff_gz += raw.gz;

      sampleCount++; // Increment sample count
      delay(2);      // Delay to maintain sampling rate
    }

    // Use the actual counted samples for offset calculation
    ofst.accX = buff_ax / sampleCount;
    ofst.accY = buff_ax / sampleCount;
    ofst.accZ = buff_ax / sampleCount - 16384;

    ofst.gyroX = buff_gx / sampleCount;
    ofst.gyroY = buff_gy / sampleCount;
    ofst.gyroZ = buff_gz / sampleCount;

    imu.setOffsets(ofst); 
    saveCalibrationValues();

    digitalWrite(2, LOW); // Turn off LED after calibration.
  }
  else
  {
    // Only print this once offsets are found and need to be in EEPROM
    printStoredCalibrationValues();

  }
#else

 
  Serial.println("Calibrating without EEPROM...");
  digitalWrite(2, HIGH); 

  unsigned long startTime = millis();
  while (millis() - startTime < cal_time)
  {
    imu.readRawIMUData(); // Collect raw data for calibration period
    buff_ax += raw.ax;
    buff_ax += raw.ay;
    buff_ax += raw.az;

    buff_gx += raw.gx;
    buff_gy += raw.gy;
    buff_gz += raw.gz;

    sampleCount++; // Increment sample count
    delay(2);      // Delay to maintain sampling rate
  }

  // Use the actual counted samples for offset calculation
  ofst.accX = buff_ax / sampleCount;
  ofst.accY = buff_ax / sampleCount;
  ofst.accZ = buff_ax / sampleCount - 16384;

  ofst.gyroX = buff_gx / sampleCount;
  ofst.gyroY = buff_gy / sampleCount;
  ofst.gyroZ = buff_gz / sampleCount;

  imu.setOffsets(ofst); 

  digitalWrite(2, LOW); // Turn off LED after calibration.

  // Print calculated offsets for debugging
  Serial.println("\nCalibration Complete.");
  Serial.println("Calculated Offsets:");
  Serial.print("Gyro X Offset: ");
  Serial.println(ofst.gyroX);
  Serial.print("Gyro Y Offset: ");
  Serial.println(ofst.gyroY);
  Serial.print("Gyro Z Offset: ");
  Serial.println(ofst.gyroZ);
  Serial.print("Acc X Offset: ");
  Serial.println(ofst.accX);
  Serial.print("Acc Y Offset: ");
  Serial.println(ofst.accY);
  Serial.print("Acc Z Offset: ");
  Serial.println(ofst.accZ);

#endif
}

void Calibration::saveCalibrationValues()
{
  const ImuOffsets& ofst = imu.getOffsets();

  EEPROM.writeLong(0, 0x12345678);
  EEPROM.writeLong(4,  ofst.accX);
  EEPROM.writeLong(8,  ofst.accY);
  EEPROM.writeLong(12, ofst.accZ);
  EEPROM.writeLong(16, ofst.gyroX);
  EEPROM.writeLong(20, ofst.gyroY);
  EEPROM.writeLong(24, ofst.gyroZ);
  // Remove: EEPROM.writeFloat(28, imu.temperatureC);
  
  uint32_t checksum = ofst.accX + ofst.accY + ofst.accZ + 
                      ofst.gyroX + ofst.gyroY + ofst.gyroZ;
  EEPROM.writeLong(28, checksum);  // Moved to address 28
  
  EEPROM.commit();
  Serial.println("\nCalibration values saved to EEPROM");
  printStoredCalibrationValues(); 
}

bool Calibration::loadCalibrationValues()
{
  ImuOffsets ofst;
  //First instance to indicate a calibration 
  if (EEPROM.readLong(0) != 0x12345678)
    return false;
  
  // Load values
  ofst.accX = EEPROM.readLong(4);
  ofst.accY = EEPROM.readLong(8);
  ofst.accZ = EEPROM.readLong(12);
  ofst.gyroX = EEPROM.readLong(16);
  ofst.gyroY = EEPROM.readLong(20);
  ofst.gyroZ = EEPROM.readLong(24);

  imu.setOffsets(ofst); 
  
  // Verify checksum
  uint32_t storedChecksum = EEPROM.readLong(28);
  uint32_t calculatedChecksum = ofst.accX + ofst.accY + ofst.accZ + 
                                ofst.gyroX + ofst.gyroY + ofst.gyroZ;
  
  if (storedChecksum != calculatedChecksum)
  {
    Serial.println("WARNING: EEPROM checksum mismatch - data may be corrupted!");
    return false;
  }
  
  Serial.println("Calibration values loaded from EEPROM (checksum verified)");
  return true;
}

// Only used if needed to get new values or writing did not go well.
void Calibration::clearCalibrationData()
{

  EEPROM.begin(EEPROM_SIZE);

  // Set a specific value to indicate that the data is cleared or invalid
  int addr = 0; 

  if (addr != 0) {
      for (int i = 1; i <= 6; i++ )
      {
        EEPROM.writeLong(addr, 0x0); 
        addr += 4;  
      }

      EEPROM.writeFloat(28, 0x0);
      EEPROM.commit(); // Make sure to commit the changes to EEPROM
      Serial.println("Calibration data cleared.");
      printStoredCalibrationValues();
  }
  
}

void Calibration::printStoredCalibrationValues()
{

  // Check for unique ID
  if (EEPROM.readLong(0) == 0x12345678)
  {
    // Read stored offset values
    int32_t storedAccXOffset  = EEPROM.readLong(4);
    int32_t storedAccYOffset  = EEPROM.readLong(8);
    int32_t storedAccZOffset  = EEPROM.readLong(12);
    int32_t storedGyroXOffset = EEPROM.readLong(16);
    int32_t storedGyroYOffset = EEPROM.readLong(20);
    int32_t storedGyroZOffset = EEPROM.readLong(24);
    // float storedTempOffset = EEPROM.readFloat(28);

    // Print formatted output
    Serial.println("\n====== Stored EEPROM Calibration Values ======");
    Serial.printf("Gyro X Offset: %ld\n", storedGyroXOffset);
    Serial.printf("Gyro Y Offset: %ld\n", storedGyroYOffset);
    Serial.printf("Gyro Z Offset: %ld\n", storedGyroZOffset);
    Serial.printf("Acc X Offset: %ld\n", storedAccXOffset);
    Serial.printf("Acc Y Offset: %ld\n", storedAccYOffset);
    Serial.printf("Acc Z Offset: %ld\n", storedAccZOffset);
    // Serial.printf("Temperature Offset: %.2f°F\n", storedTempOffset);
    Serial.println("================================================\n");
  }
  else
  {
    Serial.println("No valid data found in EEPROM");
  }
}