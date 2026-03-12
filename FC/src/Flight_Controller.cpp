#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include <Wire.h>
#include <EEPROM.h>
#include "Flight_Controller.h"
#include "PID_Webserver.h"
#include "ReceiverInput.h"
#include "Calibration.h"
#include "IMU.h"

void FC::initialize()
{
  // Bring up I²C before touching the MPU 
  imu.initializeI2CBus();
  setupInputPins();
  //Perform MPU calibration. 
  Calibration cal(imu); cal.CalibrateIMU();
  //Initialize and arm ESCs 
  motors.Initialize_ESCs();
  
  #ifndef USE_EEPROM
   cal.clearCalibrationData();
  #endif

}

void FC::run(){
  imu.readRawIMUData(); 
  imu.scaleIMU(); 

  FilteredAttitude filt_gyro = imu.compFilter(imu.getScaledData());

  const ReceiverPulseSnapshot input = ReadInput();
  updateState(input.throttle, input.yaw);

  if (state == RUNNING)
  {
      computeControlSetpoints(input.roll, input.pitch, input.throttle, input.yaw);
      pid.calculate_pid(imu.getScaledData());

      motors.mix_motors(input.throttle, pid.getOutput());
      motors.write_motors();
  }
  else {
      // keep motors off and reset PID
      motors.idle();
      if (state == OFF) pid.reset();
  }

}

// Maps stick µs inputs to rate setpoints in deg/s.
// Dead zone: 1492–1508µs. Max deflection ~492µs → ~164 deg/s at factor=3.
// TODO: auto-level (angle outer loop) will add a rate correction on top of these
//       setpoints once the rate PID is tuned — keep it separate, not in here.
// TODO: bumpless start — on first arm, seed setpoints to current gyro rates so
//       initial PID error is ~zero (see PID::reset).
void FC::computeControlSetpoints(const int Roll, const int Pitch, const int Throttle, const int Yaw)
{
  PIDSetpoints sp = {};
  const float factor = 3.0f; // µs-delta → deg/s  (492µs / 3 = 164 deg/s max)

  if      (Roll  > 1508) sp.roll  = (Roll  - 1508) / factor;
  else if (Roll  < 1492) sp.roll  = (Roll  - 1492) / factor;

  if      (Pitch > 1508) sp.pitch = (Pitch - 1508) / factor;
  else if (Pitch < 1492) sp.pitch = (Pitch - 1492) / factor;

  if (Throttle > 998)
  {
    if      (Yaw > 1606) sp.yaw = (Yaw - 1606) / factor;
    else if (Yaw < 1492) sp.yaw = (Yaw - 1492) / factor;
  }

  //update setpoints for PID controller
  pid.setSetpoints(sp);
}

void FC::updateState(int throttle, int yaw)
{
  
  unsigned long currentTime = millis();

  // Start condition (state = START)
  if (throttle < 1065 && yaw < 1050)
  {
    if (!isDebounceConditionMet)
    {
      lastDebounceTime = currentTime;
      isDebounceConditionMet = true;
    }
    else if ((currentTime - lastDebounceTime) > debounceDelay && state == OFF)
    {
      state = START;
      isDebounceConditionMet = false; // Reset for next condition
    }
  }

  // Transition to start = 2 (running)
  if (state == START && throttle < 1550 && yaw > 1450)
  {
    if (!isDebounceConditionMet)
    {
      lastDebounceTime = currentTime;
      isDebounceConditionMet = true;
    }
    else if ((currentTime - lastDebounceTime) > debounceDelay)
    {
      state = RUNNING;
      isDebounceConditionMet = false; // Reset for next condition
    }
  }

  // Turn off motors (start = 0)
  if (state == RUNNING && throttle <= 1064 && yaw > 1976)
  {
    if (!isDebounceConditionMet)
    {
      lastDebounceTime = currentTime;
      isDebounceConditionMet = true;
    }
    else if ((currentTime - lastDebounceTime) > debounceDelay)
    {
      state = OFF;                      // Turn off
      isDebounceConditionMet = false; // Reset for next condition
    }
  }

  // Reset debounce condition if none of the above conditions are met
  if (!(throttle < 1065 && yaw < 1050) &&
      !(state == START && throttle < 1550 && yaw > 1450) &&
      !(state == RUNNING && throttle <= 1064 && yaw > 1976))
  {
    isDebounceConditionMet = false;
  }
}