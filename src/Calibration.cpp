#include <Arduino.h>
#include <EEPROM.h>
#include "Calibration.h"
#include "Flight_Controller.h"

Calibration cal;

void Calibration::saveCalibrationValues()
{
  if (EEPROM.readLong(0) != 0x12345678)
  {
    EEPROM.writeLong(0, 0x12345678); // Unique identifier indicating values have been saved
    EEPROM.writeLong(4, flightController.accXOffset);
    EEPROM.writeLong(8, flightController.accYOffset);
    EEPROM.writeLong(12, flightController.accZOffset);
    EEPROM.writeLong(16, flightController.gyroXOffset);
    EEPROM.writeLong(20, flightController.gyroYOffset);
    EEPROM.writeLong(24, flightController.gyroZOffset);
    EEPROM.writeFloat(28, flightController.temperatureC);
    EEPROM.commit();
    Serial.println("Calibration values saved to EEPROM");
  }
  else
  {
    Serial.println("EEPROM already contains calibration values. Skipping save.");
  }
}

bool Calibration::loadCalibrationValues()
{
  if (EEPROM.readLong(0) != 0x12345678)
  {
    return false; // Calibration data not found or not valid
  }
  // If the unique ID matches, proceed to read calibration data
  flightController.accXOffset = EEPROM.readLong(4);
  flightController.accYOffset = EEPROM.readLong(8);
  flightController.accZOffset = EEPROM.readLong(12);
  flightController.gyroXOffset = EEPROM.readLong(16);
  flightController.gyroYOffset = EEPROM.readLong(20);
  flightController.gyroZOffset = EEPROM.readLong(24);
  flightController.temperatureC = EEPROM.readFloat(28);
  return true;
}

// Only used if needed to get new values or writing did not go well.
void Calibration::clearCalibrationData()
{
  EEPROM.begin(flightController.EEPROM_SIZE);
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
  const int gyroXOffsetAddr = 4;
  const int gyroYOffsetAddr = 8;
  const int gyroZOffsetAddr = 12;
  const int accXOffsetAddr = 16;
  const int accYOffsetAddr = 20;
  const int accZOffsetAddr = 24;
  const int tempOffsetAddr = 28;

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
    float storedTempOffset = EEPROM.readFloat(tempOffsetAddr);

    // Print formatted output
    Serial.println("\n====== Stored Calibration Values ======");
    Serial.printf("Gyro X Offset: %ld\n", storedGyroXOffset);
    Serial.printf("Gyro Y Offset: %ld\n", storedGyroYOffset);
    Serial.printf("Gyro Z Offset: %ld\n", storedGyroZOffset);
    Serial.printf("Acc X Offset: %ld\n", storedAccXOffset);
    Serial.printf("Acc Y Offset: %ld\n", storedAccYOffset);
    Serial.printf("Acc Z Offset: %ld\n", storedAccZOffset);
    Serial.printf("Temperature Offset: %.2f°C\n", storedTempOffset);
    Serial.println("=======================================\n");
  }
  else
  {
    Serial.println("No valid data found in EEPROM");
  }
}