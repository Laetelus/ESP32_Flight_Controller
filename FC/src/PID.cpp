
#include "Flight_Controller.h"

void FC::calculate_pid()
{
  // Roll calculations
  pid_error_temp = pid_roll_setpoint - gyro_roll_input; // Correct error calculation direction
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;


  // Constrain integral memory (anti-windup)
  if (pid_i_mem_roll > pid_max_roll)
    pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < -pid_max_roll)
    pid_i_mem_roll = -pid_max_roll;

  // PID output calculation including proportional, integral, and derivative terms
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);

  // Constrain PID output
  if (pid_output_roll > pid_max_roll)
    pid_output_roll = pid_max_roll;
  else if (pid_output_roll < -pid_max_roll)
    pid_output_roll = -pid_max_roll;

  pid_last_roll_d_error = pid_error_temp;

  // Pitch calculations
  pid_error_temp = pid_pitch_setpoint - gyro_pitch_input; // Correct error calculation direction
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;

  // Constrain integral memory (anti-windup)
  if (pid_i_mem_pitch > pid_max_pitch)
    pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < -pid_max_pitch)
    pid_i_mem_pitch = -pid_max_pitch;

  // PID output calculation including proportional, integral, and derivative terms
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);

  // Constrain PID output
  if (pid_output_pitch > pid_max_pitch)
    pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < -pid_max_pitch)
    pid_output_pitch = -pid_max_pitch;

  pid_last_pitch_d_error = pid_error_temp;

  // Yaw calculations
  pid_error_temp = pid_yaw_setpoint - gyro_yaw_input; // Correct error calculation direction
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;

  // Constrain integral memory (anti-windup)
  if (pid_i_mem_yaw > pid_max_yaw)
    pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < -pid_max_yaw)
    pid_i_mem_yaw = -pid_max_yaw;

  // PID output calculation including proportional, integral, and derivative terms
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);

  // Constrain PID output
  if (pid_output_yaw > pid_max_yaw)

    pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < -pid_max_yaw)
    pid_output_yaw = -pid_max_yaw;

  pid_last_yaw_d_error = pid_error_temp;

  // ─── Deadband to kill tiny noise ─────────────────────────────
  if (fabs(pid_output_roll)  < 1.0) pid_output_roll  = 0.0;
  if (fabs(pid_output_pitch) < 1.0) pid_output_pitch = 0.0;
  if (fabs(pid_output_yaw)   < 1.0) pid_output_yaw   = 0.0;
// ─────────────────────────────────────────────────────────────


}

void FC::Reset_PID()
{
  const AccelAngleData& ang = imu.getAccelAngles();

  start = 2;
  // Should set this as zero initially 
  // its good when we start on the ground to have a bumpless start. 
  pid_roll_setpoint  = ang.Roll; 
  pid_pitch_setpoint = ang.Pitch;
  pid_yaw_setpoint   = gyro_yaw_input; 
  
  // reset PID state
  pid_i_mem_roll        = pid_last_roll_d_error  = 0;
  pid_i_mem_pitch       = pid_last_pitch_d_error = 0;
  pid_i_mem_yaw         = pid_last_yaw_d_error   = 0;

}