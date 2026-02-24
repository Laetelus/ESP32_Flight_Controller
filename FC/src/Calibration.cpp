#include <Arduino.h>
#include <EEPROM.h>
#include "Calibration.h"
#include "Flight_Controller.h"

FC fc; 

// #define USE_EEPROM

void Calibration::performCalibration()
{
  // Run for 2 seconds, store as the sample count 
  unsigned long cal_time = 2000; 
  long buff_ax = 0, buff_ay = 0,
       buff_az = 0, buff_gx = 0,
       buff_gy = 0, buff_gz = 0;

  int sampleCount = 0;

#ifdef USE_EEPROM

  EEPROM.begin(fc.EEPROM_SIZE);

  if (!cal.loadCalibrationValues())
  {

    Serial.println("Calibration data not found in EEPROM. Calibrating...");
    // Blink LED to indicate calibration is in progress
    digitalWrite(2, HIGH); // Turn on LED to indicate start of calibration

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
    fc.accZOffset = buff_ax / sampleCount - 16384;

    fc.gyroXOffset = buff_gx / sampleCount;
    fc.gyroYOffset = buff_gy / sampleCount;
    fc.gyroZOffset = buff_gz / sampleCount;

    cal.saveCalibrationValues();

    digitalWrite(2, LOW); // Turn off LED after calibration.
  }
  else
  {
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
  fc.accZOffset = buff_ax / sampleCount - 16384;

  fc.gyroXOffset = buff_gx / sampleCount;
  fc.gyroYOffset = buff_gy / sampleCount;
  fc.gyroZOffset = buff_gz / sampleCount;

  digitalWrite(2, LOW); // Turn off LED after calibration.

  // Print calculated offsets for debugging
  Serial.println("\nCalibration Complete.");
  Serial.println("Calculated Offsets:");
  Serial.print("Gyro X Offset: ");
  Serial.println(fc.gyroXOffset);
  Serial.print("Gyro Y Offset: ");
  Serial.println(fc.gyroYOffset);
  Serial.print("Gyro Z Offset: ");
  Serial.println(fc.gyroZOffset);
  Serial.print("Acc X Offset: ");
  Serial.println(fc.accXOffset);
  Serial.print("Acc Y Offset: ");
  Serial.println(fc.accYOffset);
  Serial.print("Acc Z Offset: ");
  Serial.println(fc.accZOffset);

#endif
}

void Calibration::saveCalibrationValues()
{
  EEPROM.writeLong(0, 0x12345678);
  EEPROM.writeLong(4, fc.accXOffset);
  EEPROM.writeLong(8, fc.accYOffset);
  EEPROM.writeLong(12, fc.accZOffset);
  EEPROM.writeLong(16, fc.gyroXOffset);
  EEPROM.writeLong(20, fc.gyroYOffset);
  EEPROM.writeLong(24, fc.gyroZOffset);
  // Remove: EEPROM.writeFloat(28, fc.temperatureC);
  
  uint32_t checksum = fc.accXOffset + fc.accYOffset + fc.accZOffset + 
                      fc.gyroXOffset + fc.gyroYOffset + fc.gyroZOffset;
  EEPROM.writeLong(28, checksum);  // Moved to address 28
  
  EEPROM.commit();
  Serial.println("\nCalibration values saved to EEPROM");
  printStoredCalibrationValues(); 
}

bool Calibration::loadCalibrationValues()
{
  //First instance to indicate a calibration 
  if (EEPROM.readLong(0) != 0x12345678)
    return false;
  
  // Load values
  fc.accXOffset = EEPROM.readLong(4);
  fc.accYOffset = EEPROM.readLong(8);
  fc.accZOffset = EEPROM.readLong(12);
  fc.gyroXOffset = EEPROM.readLong(16);
  fc.gyroYOffset = EEPROM.readLong(20);
  fc.gyroZOffset = EEPROM.readLong(24);
  
  // Verify checksum
  uint32_t storedChecksum = EEPROM.readLong(28);
  uint32_t calculatedChecksum = fc.accXOffset + fc.accYOffset + fc.accZOffset + 
                                fc.gyroXOffset + fc.gyroYOffset + fc.gyroZOffset;
  
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

  EEPROM.begin(fc.EEPROM_SIZE);

  // Set a specific value to indicate that the data is cleared or invalid
  int addr = 0; 

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