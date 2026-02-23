#include <Arduino.h>
#include <EEPROM.h>
#include "Calibration.h"
#include "Flight_Controller.h"

FC fc; 

void Calibration::performCalibration()
{
  // Run for 2 seconds, store as the sample count 
  unsigned long cal_time = 2000; 
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
      processIMUData(); // Collect raw data for calibration period
      buff_ax += raw_ax;
      buff_ay += raw_ay;
      buff_az += raw_az;

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
    // Only print this once offsets are found and need to be in EEPROM
    cal.printStoredCalibrationValues();

  }
#else

  Serial.println("Calibrating without EEPROM...");
  digitalWrite(2, HIGH); 

  unsigned long startTime = millis();
  while (millis() - startTime < cal_time)
  {
    fc.processIMUData(); // Collect raw data for calibration period
    buff_ax += fc.raw_ax;
    buff_ax += fc.raw_ay;
    buff_ax += fc.raw_az;

    buff_gx += fc.raw_gx;
    buff_gy += fc.raw_gy;
    buff_gz += fc.raw_gz;

    sampleCount++; // Increment sample count
    delay(2);      // Delay to maintain sampling rate
  }

  // Use the actual counted samples for offset calculation
  fc.accXOffset = buff_ax / sampleCount;
  fc.accYOffset = buff_ax / sampleCount;
  fc.accZOffset = buff_ax / sampleCount;

  fc.gyroXOffset = buff_gx / sampleCount;
  fc.gyroYOffset = buff_gy / sampleCount;
  fc.gyroZOffset = buff_gz / sampleCount;

  digitalWrite(2, LOW); // Turn off LED after calibration.

  // Print calculated offsets for debugging
  Serial.println("\nCalibration Complete.");
  Serial.println("Calculated Offsets:");
  Serial.print("Acc X Offset: ");
  Serial.println(fc.accXOffset);
  Serial.print("Acc Y Offset: ");
  Serial.println(fc.accYOffset);
  Serial.print("Acc Z Offset: ");
  Serial.println(fc.accZOffset);
  Serial.print("Gyro X Offset: ");
  Serial.println(fc.gyroXOffset);
  Serial.print("Gyro Y Offset: ");
  Serial.println(fc.gyroYOffset);
  Serial.print("Gyro Z Offset: ");
  Serial.println(fc.gyroZOffset);

#endif
}

void Calibration::saveCalibrationValues()
{
  if (EEPROM.readLong(0) != 0x12345678)
  {
    EEPROM.writeLong(0, 0x12345678); // Unique identifier indicating values have been saved
    EEPROM.writeLong(4, fc.accXOffset);
    EEPROM.writeLong(8, fc.accYOffset);
    EEPROM.writeLong(12, fc.accZOffset);
    EEPROM.writeLong(16, fc.gyroXOffset);
    EEPROM.writeLong(20, fc.gyroYOffset);
    EEPROM.writeLong(24, fc.gyroZOffset);
    EEPROM.writeFloat(28, fc.temperatureC);
    EEPROM.commit();
    Serial.println("Calibration values saved to EEPROM");
    printStoredCalibrationValues(); 
  }
  else
  {
    Serial.println("EEPROM already contains calibration values. Skipping save.");
    printStoredCalibrationValues(); 
  }
}

bool Calibration::loadCalibrationValues()
{
  if (EEPROM.readLong(0) != 0x12345678)
  {
    return false; // Calibration data not found or not valid
  }
  // If the unique ID matches, proceed to read calibration data
  fc.accXOffset = EEPROM.readLong(4);
  fc.accYOffset = EEPROM.readLong(8);
  fc.accZOffset = EEPROM.readLong(12);
  fc.gyroXOffset = EEPROM.readLong(16);
  fc.gyroYOffset = EEPROM.readLong(20);
  fc.gyroZOffset = EEPROM.readLong(24);
  fc.temperatureC = EEPROM.readFloat(28);
  return true;
}

// Only used if needed to get new values or writing did not go well.
void Calibration::clearCalibrationData()
{
  EEPROM.begin(fc.EEPROM_SIZE);
  // Set a specific value to indicate that the data is cleared or invalid
  long invalidValue = 0x0;

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

void Calibration::printStoredCalibrationValues()
{

  // Addresses where the calibration values are stored
  const int gyroXOffsetAddr =  0;
  const int gyroYOffsetAddr =  0;
  const int gyroZOffsetAddr =  0;
  const int accXOffsetAddr  =  0;
  const int accYOffsetAddr  =  0;
  const int accZOffsetAddr  =  0;
  //const int tempOffsetAddr = 28;

  // Check for unique ID
  if (EEPROM.readLong(0) == 0x12345678)
  {
    // Read stored offset values
    int32_t storedGyroXOffset = EEPROM.readLong(gyroXOffsetAddr);
    int32_t storedGyroYOffset = EEPROM.readLong(gyroYOffsetAddr);
    int32_t storedGyroZOffset = EEPROM.readLong(gyroZOffsetAddr);
    int32_t storedAccXOffset = EEPROM.readLong(accXOffsetAddr);
    int32_t storedAccYOffset = EEPROM.readLong(accYOffsetAddr);
    int32_t storedAccZOffset = EEPROM.readLong(accZOffsetAddr);
   // float storedTempOffset = EEPROM.readFloat(tempOffsetAddr);

    // Print formatted output
    Serial.println("\n====== Stored Calibration Values ======");
    Serial.printf("Gyro X Offset: %ld\n", storedGyroXOffset);
    Serial.printf("Gyro Y Offset: %ld\n", storedGyroYOffset);
    Serial.printf("Gyro Z Offset: %ld\n", storedGyroZOffset);
    Serial.printf("Acc X Offset: %ld\n", storedAccXOffset);
    Serial.printf("Acc Y Offset: %ld\n", storedAccYOffset);
    Serial.printf("Acc Z Offset: %ld\n", storedAccZOffset);
    //Serial.printf("Temperature Offset: %.2f°C\n", storedTempOffset);
    Serial.println("=======================================\n");
  }
  else
  {
    Serial.println("No valid data found in EEPROM");
  }
}