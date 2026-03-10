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

// IMU imu;


// Flight controller interface
void FC::initialize()
{

  // Bring up I²C before touching the MPU 
  imu.initializeI2CBus();
  setupInputPins();
  //Perform MPU calibration. 
  Calibration cal(imu); cal.performCalibration();
  // finally Initialize and arm ESCs 
  Initialize_ESCs();
  
  //Clears calibration values 
  #ifndef USE_EEPROM
   cal.clearCalibrationData();
  #endif

}

void FC::run(){
  imu.processIMUData(); 
  imu.scaleIMU(); 
  motorControls(); 
  mix_motors();
}

//wtf are you doing here??? reread brooks FC and redo this again please 
void FC::computeControlSetpoints(int &Roll, int &Pitch, int &Throttle, int &Yaw)
{

  const AccelAngleData& ang = imu.getAccelAngles();

  const float angle_deadband      = 2.0f;   // ignore <2°
  const float level_adjust_factor = 0.10f;  // gentler pull

  float errorRoll = fabs(ang.Roll)  > angle_deadband ? ang.Roll  : 0;
  float errorPitch= fabs(ang.Pitch) > angle_deadband ? ang.Pitch : 0;

  roll_level_adjust  = errorRoll  * level_adjust_factor;
  pitch_level_adjust = errorPitch * level_adjust_factor;


  // If auto-level is disabled, set adjustments to zero
  if (!auto_level)
  {
    pitch_level_adjust = 0;
    roll_level_adjust = 0;
  }

  // Initialize desired rates
  float desired_rate_roll = 0;
  float desired_rate_pitch = 0;

  // Calculate desired rates based on user inputs for roll
  if (Roll > 1508)
  {
    desired_rate_roll = Roll - 1508;
  }
  else if (Roll < 1492)
  {
    desired_rate_roll = Roll - 1492;
  }
  
  // Calculate desired rates based on user inputs for pitch
  if (Pitch > 1508)
  {
    desired_rate_pitch = Pitch - 1508;
  }
  else if (Pitch < 1492)
  {
    desired_rate_pitch = Pitch - 1492;
  }

  // Adjust rates for level flight if auto-level is enabled
  if (auto_level)
  {
    desired_rate_roll -= roll_level_adjust;
    desired_rate_pitch -= pitch_level_adjust;
  }

  // The factor translates the receiver input into a rate in degrees per second
  float factor = 3.0; 

  // Set the PID setpoints based on desired rates and factor
  pid_roll_setpoint = desired_rate_roll / factor;
  pid_pitch_setpoint = desired_rate_pitch / factor;

  // Set the PID set point for yaw based on user inputs
  pid_yaw_setpoint = 0;
  if (Throttle > 998)
  {
    if (Yaw > 1606)
    {
      pid_yaw_setpoint = (Yaw - 1606) / factor;
    }
    else if (Yaw < 1492)
    {
      pid_yaw_setpoint = (Yaw - 1492) / factor;
    }
  }
}






