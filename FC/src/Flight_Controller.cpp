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

void FC::initialize_FC()
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

  // Update complementary filter — stores result in imu.attitude_ for auto-level.
  imu.compFilter(imu.getScaledData());

  const ReceiverPulseSnapshot input = ReadInput();
  lastInput_ = input; // Store the last input for printing
  updateState(input.throttle, input.yaw);

  if (state == RUNNING)
  {
      computeControlSetpoints(input.roll, input.pitch, input.throttle, input.yaw);

      // Only integrate I-term when motors are actually spinning (~>1100µs).
      // Below spin threshold there is no physical correction happening, so
      // accumulating I-term against an error that can't be corrected causes
      // windup that will kick the drone on takeoff.
      const bool spooled = input.throttle > 1100;
      pid.calculate_pid(imu.getScaledData(), spooled);

      motors.mix_motors(input.throttle, pid.getOutput());
      motors.write_motors();
  }
  else {
      motors.idle();
      // Reset PID whenever not running — prevents I-term windup accumulated
      // during bench testing from carrying over into the next arm.
      pid.reset();
  }

}
// Maps stick µs inputs to rate setpoints in deg/s.
// Dead zone: 1492–1508µs. Max deflection ~492µs → ~164 deg/s at factor=3.
//
// Auto-level outer loop: when auto_level is true, the current tilt angle
// (from the complementary filter) is multiplied by 15 and subtracted from
// the stick deflection before dividing by 3. This injects a corrective rate
// proportional to tilt — e.g. 10° tilt → 50 deg/s correction — so the drone
// returns to level whenever the sticks are centred.
void FC::computeControlSetpoints(const int Roll, const int Pitch, const int Throttle, const int Yaw)
{
  PIDSetpoints sp = {};
  const float factor = 3.0f; // µs-delta → deg/s  (492µs / 3 = 164 deg/s max)

  float roll_level_adjust  = 0.0f;
  float pitch_level_adjust = 0.0f;
  if (auto_level)
  {
    const FilteredAttitude& att = imu.getAttitude();
    roll_level_adjust  = att.roll_deg  * 15.0f;
    pitch_level_adjust = att.pitch_deg * 15.0f;
  }

  // Compute stick delta (zero inside dead zone), then subtract angle correction.
  float stick_roll  = 0.0f;
  float stick_pitch = 0.0f;
  if      (Roll  > 1508) stick_roll  = Roll  - 1508;
  else if (Roll  < 1492) stick_roll  = Roll  - 1492;
  if      (Pitch > 1508) stick_pitch = Pitch - 1508;
  else if (Pitch < 1492) stick_pitch = Pitch - 1492;

  sp.roll  = (stick_roll  - roll_level_adjust)  / factor;
  sp.pitch = (stick_pitch - pitch_level_adjust) / factor;

  // Yaw: gate on throttle > 1050 so a yaw stick input can't interfere with
  // the disarm sequence (throttle ≤ 1064 && yaw right). Symmetric dead zone
  // 1492–1508 matches roll/pitch — max yaw rate ≈ 164 deg/s.
  if (Throttle > 1050)
  {
    if      (Yaw > 1508) sp.yaw = (Yaw - 1508) / factor;
    else if (Yaw < 1492) sp.yaw = (Yaw - 1492) / factor;
  }

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
      // Sync gyro-integrated angles to the accel reference so auto-level
      // starts from a known-good baseline (drone may have been moved before arming).
      imu.syncAttitudeToAccel();
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