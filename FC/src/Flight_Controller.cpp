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
  Calibration cal(imu); cal.performCalibration();
  //Initialize and arm ESCs 
  motors.Initialize_ESCs();
  
  //Clears calibration values 
  #ifndef USE_EEPROM
   cal.clearCalibrationData();
  #endif

}

void FC::run(){
  imu.readRawIMUData(); 
  imu.scaleIMU(); 
  //TOOD: we'll need to add the comp filter in the future. 
  const ControlInput input = updateState();

  if (state == RUNNING)
  {
    int roll = input.roll;
    int pitch = input.pitch;
    int throttle = input.throttle;
    int yaw = input.yaw;
    // computeControlSetpoints(roll, pitch, throttle, yaw);
    // pid.calculate_pid();
  }

  // test PID gains data parased from webserver in a readable format 
  // const PIDgains gains = pid.getGains();
  // Serial.println("Current PID Gains:");
  // Serial.print("P Gain Roll: "); Serial.println(gains.p_gain_roll, 3); 
  // Serial.print("I Gain Roll: "); Serial.println(gains.i_gain_roll, 3);
  // Serial.print("D Gain Roll: "); Serial.println(gains.d_gain_roll, 3);
  // Serial.println();
  // Serial.print("P Gain Pitch: "); Serial.println(gains.p_gain_pitch, 3);
  // Serial.print("I Gain Pitch: "); Serial.println(gains.i_gain_pitch, 3);
  // Serial.print("D Gain Pitch: "); Serial.println(gains.d_gain_pitch, 3);
  // Serial.print("P Gain Yaw: "); Serial.println(gains.p_gain_yaw, 3);
  // Serial.print("I Gain Yaw: "); Serial.println(gains.i_gain_yaw, 3);
  // Serial.print("D Gain Yaw: "); Serial.println(gains.d_gain_yaw, 3);
  // Serial.println();
  

  const PIDOut pid_Output = pid.getOutput();
  motors.mix_motors(input.throttle, pid_Output, state);
  motors.write_motors(); 
}

// //wtf are you doing here??? reread brooks FC and redo this again please 
// void FC::computeControlSetpoints(int &Roll, int &Pitch, int &Throttle, int &Yaw)
// {

//   const AccelAngleData& ang = imu.getAccelAngles();

//   const float angle_deadband      = 2.0f;   // ignore <2°
//   const float level_adjust_factor = 0.10f;  // gentler pull

//   float errorRoll  = fabs(ang.Roll)  > angle_deadband ? ang.Roll  : 0;
//   float errorPitch = fabs(ang.Pitch) > angle_deadband ? ang.Pitch : 0;

//   roll_level_adjust  = errorRoll  * level_adjust_factor;
//   pitch_level_adjust = errorPitch * level_adjust_factor;


//   // If auto-level is disabled, set adjustments to zero
//   if (!auto_level)
//   {
//     pitch_level_adjust = 0;
//     roll_level_adjust = 0;
//   }

//   // Initialize desired rates
//   float desired_rate_roll = 0;
//   float desired_rate_pitch = 0;

//   // Calculate desired rates based on user inputs for roll
//   if (Roll > 1508)
//   {
//     desired_rate_roll = Roll - 1508;
//   }
//   else if (Roll < 1492)
//   {
//     desired_rate_roll = Roll - 1492;
//   }
  
//   // Calculate desired rates based on user inputs for pitch
//   if (Pitch > 1508)
//   {
//     desired_rate_pitch = Pitch - 1508;
//   }
//   else if (Pitch < 1492)
//   {
//     desired_rate_pitch = Pitch - 1492;
//   }

//   // Adjust rates for level flight if auto-level is enabled
//   if (auto_level)
//   {
//     desired_rate_roll -= roll_level_adjust;
//     desired_rate_pitch -= pitch_level_adjust;
//   }

//   // The factor translates the receiver input into a rate in degrees per second
//   float factor = 3.0; 

//   // Set the PID setpoints based on desired rates and factor
//   pid_roll_setpoint = desired_rate_roll / factor;
//   pid_pitch_setpoint = desired_rate_pitch / factor;

//   // Set the PID set point for yaw based on user inputs
//   pid_yaw_setpoint = 0;
//   if (Throttle > 998)
//   {
//     if (Yaw > 1606)
//     {
//       pid_yaw_setpoint = (Yaw - 1606) / factor;
//     }
//     else if (Yaw < 1492)
//     {
//       pid_yaw_setpoint = (Yaw - 1492) / factor;
//     }
//   }
// }

ControlInput FC::updateState()
{
  
  const ReceiverPulseSnapshot input = getReceiverPulseSnapshot();
  int roll = static_cast<int>(input.roll);
  int pitch = static_cast<int>(input.pitch);
  int throttle = static_cast<int>(input.throttle);
  int yaw = static_cast<int>(input.yaw);

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

  // Transition to start = 2 (running), handled inside 
  if (state == START && throttle < 1550 && yaw > 1450)
  {
    if (!isDebounceConditionMet)
    {
      lastDebounceTime = currentTime;
      isDebounceConditionMet = true;
    }
    else if ((currentTime - lastDebounceTime) > debounceDelay)
    {
      // Reset_PID();
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

  // Serial.print("state: "); Serial.print(state);
  // Serial.print(" | roll: "); Serial.print(roll);    
  // Serial.print(" | pitch: "); Serial.print(pitch);
  // Serial.print(" | throttle: "); Serial.print(throttle);
  // Serial.print(" | yaw: "); Serial.println(yaw);

  return ControlInput{roll, pitch, throttle, yaw};
}




