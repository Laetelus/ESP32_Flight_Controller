
#include "Flight_Controller.h"

void PID::calculate_pid(const ScaledImuData& gyro, bool integrate)
{
  PIDgains PID = getGains();
  const PIDLimits PID_max = pid_max;
  PIDOut PID_out = getOutput();
  PIDMem PID_mem = pid_mem;
  float pid_error_temp;

  // Roll — error = desired rate - gyro rate
  pid_error_temp = pid_setpoint.roll - gyro.gx_dps;
  if (integrate) PID_mem.i_mem_roll += PID.i_gain_roll * pid_error_temp;
  if (PID_mem.i_mem_roll >  PID_max.roll) PID_mem.i_mem_roll =  PID_max.roll;
  else if (PID_mem.i_mem_roll < -PID_max.roll) PID_mem.i_mem_roll = -PID_max.roll;

  PID_out.roll = PID.p_gain_roll * pid_error_temp + PID_mem.i_mem_roll + PID.d_gain_roll * (pid_error_temp - PID_mem.last_roll_d_error);
  if (PID_out.roll >  PID_max.roll) PID_out.roll =  PID_max.roll;
  else if (PID_out.roll < -PID_max.roll) PID_out.roll = -PID_max.roll;
  PID_mem.last_roll_d_error = pid_error_temp;

  // Pitch — error = desired rate - gyro rate
  pid_error_temp = pid_setpoint.pitch - gyro.gy_dps;
  if (integrate) PID_mem.i_mem_pitch += PID.i_gain_pitch * pid_error_temp;
  if (PID_mem.i_mem_pitch >  PID_max.pitch) PID_mem.i_mem_pitch =  PID_max.pitch;
  else if (PID_mem.i_mem_pitch < -PID_max.pitch) PID_mem.i_mem_pitch = -PID_max.pitch;

  PID_out.pitch = PID.p_gain_pitch * pid_error_temp + PID_mem.i_mem_pitch + PID.d_gain_pitch * (pid_error_temp - PID_mem.last_pitch_d_error);
  if (PID_out.pitch >  PID_max.pitch) PID_out.pitch =  PID_max.pitch;
  else if (PID_out.pitch < -PID_max.pitch) PID_out.pitch = -PID_max.pitch;
  PID_mem.last_pitch_d_error = pid_error_temp;

  // Yaw — error = desired rate - gyro rate, wrap to ±180
  pid_error_temp = pid_setpoint.yaw - gyro.gz_dps;

  if      (pid_error_temp >  180) pid_error_temp -= 360;
  else if (pid_error_temp < -179) pid_error_temp += 360;

  if (integrate) PID_mem.i_mem_yaw += PID.i_gain_yaw * pid_error_temp;
  if (PID_mem.i_mem_yaw >  PID_max.yaw) PID_mem.i_mem_yaw =  PID_max.yaw;
  else if (PID_mem.i_mem_yaw < -PID_max.yaw) PID_mem.i_mem_yaw = -PID_max.yaw;

  PID_out.yaw = PID.p_gain_yaw * pid_error_temp + PID_mem.i_mem_yaw + PID.d_gain_yaw * (pid_error_temp - PID_mem.last_yaw_d_error);
  if (PID_out.yaw >  PID_max.yaw) PID_out.yaw =  PID_max.yaw;
  else if (PID_out.yaw < -PID_max.yaw) PID_out.yaw = -PID_max.yaw;
  PID_mem.last_yaw_d_error = pid_error_temp;

  // Deadband — suppress noise below 1 deg/s
  if (fabs(PID_out.roll)  < 1.0f) PID_out.roll  = 0.0f;
  if (fabs(PID_out.pitch) < 1.0f) PID_out.pitch = 0.0f;
  if (fabs(PID_out.yaw)   < 1.0f) PID_out.yaw   = 0.0f;

  pid_mem = PID_mem;
  pid_output = PID_out;
}

/* 
Purpose: Reset PID integral and derivative memory to 
        prevent windup and ensure smooth control when starting or stopping the motors. 
        This is especially important to avoid sudden jumps in motor output when arming the drone after it has been idle, 
        as the PID controller will start with zero error and no accumulated integral, providing a more stable takeoff.
*/
void PID::reset()
{
  pid_mem = {};
  pid_output = {};
}