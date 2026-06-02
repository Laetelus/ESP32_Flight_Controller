
#include "Flight_Controller.h"

// ============================================================
//  PID Controller — calculate_pid()
//
//  Textbook formula:  u = (Kp * e) + (Ki * ∫e dt) + (Kd * de/dt)
//
//  Solved manually each loop tick:
//    Step 1 — Error:       e        = setpoint − actual
//    Step 2 — Proportional: P       = Kp * e
//    Step 3 — Integral:    ∫e dt   += e  (running sum of past error)
//                           I       = Ki * ∫e dt
//    Step 4 — Derivative:  de/dt   = (e − e_prev)    ← Δe per tick
//                           D       = Kd * de/dt
//    Step 5 — Output:      u       = P + I + D
//
//  Implementation note:
//    dt is fixed (constant loop rate), so it is pre-baked into the gains
//    at tune time:  i_gain = Ki * dt,  d_gain = Kd / dt
//    This keeps per-loop math lean without changing the formula's meaning.
// ============================================================
void PID::calculate_pid(const ScaledImuData& gyro, bool integrate)
{
  PIDgains PID     = getGains();
  PIDOut   PID_out = getOutput();
  PIDMem   PID_mem = pid_mem;

  float e;             // Step 1 — current error
  float P, I, D;       // Step 2/3/4 — individual terms, summed in Step 5

  // ── ROLL ─────────────────────────────────────────────────────────────
  //
  //  Step 1 — Error: measure how far off the actual roll rate is from the target.
  //           The further off, the larger e is — giving P more to work with.
  //           e > 0: spinning too slow (or wrong way)  |  e < 0: spinning too fast
  e = pid_setpoint.roll - gyro.gx_dps;  // e = setpoint − actual

  //  Step 2 — Proportional: scale the error directly into a correction.
  //           Large error  → large P  → strong correction.
  //           Small error  → small P  → gentle correction.
  //           Zero error   → P = 0   → no correction needed.
  P = PID.p_gain_roll * e;              // P = Kp * e

  //  Step 3 — Integral: accumulate error over time (disabled while disarmed
  //           to prevent windup — a large I before takeoff causes a spike)
  if (integrate) PID_mem.i_mem_roll += PID.i_gain_roll * e;  // ∫e dt += e
  PID_mem.i_mem_roll = constrain(PID_mem.i_mem_roll, -max_roll, max_roll); // anti-windup clamp
  I = PID_mem.i_mem_roll;

  //  Step 4 — Derivative: how fast is the error changing? (braking term)
  //           de/dt ≈ (e_now − e_prev) / dt  →  Kd/dt baked into d_gain
  D = PID.d_gain_roll * (e - PID_mem.last_roll_d_error);
  PID_mem.last_roll_d_error = e;   // save e so next tick can compute Δe

  //  Step 5 — Output: u = P + I + D
  PID_out.roll = P + I + D;
  PID_out.roll = constrain(PID_out.roll, -max_roll, max_roll);

  // ── PITCH ─────────────────────────────────────────────────────────────
  //
  //  Step 1 — Error: same idea as roll, but on the pitch axis.
  e = pid_setpoint.pitch - gyro.gy_dps;  // e = setpoint − actual

  //  Step 2 — Proportional: how far off we are drives how hard we correct.
  P = PID.p_gain_pitch * e;              // P = Kp * e

  //  Step 3
  if (integrate) PID_mem.i_mem_pitch += PID.i_gain_pitch * e;
  PID_mem.i_mem_pitch = constrain(PID_mem.i_mem_pitch, -max_pitch, max_pitch);
  I = PID_mem.i_mem_pitch;

  //  Step 4
  D = PID.d_gain_pitch * (e - PID_mem.last_pitch_d_error);
  PID_mem.last_pitch_d_error = e;

  //  Step 5
  PID_out.pitch = P + I + D;
  PID_out.pitch = constrain(PID_out.pitch, -max_pitch, max_pitch);

  // ── YAW ───────────────────────────────────────────────────────────────
  //
  //  Step 1 — Error: same idea, but yaw wraps around 360°.
  //           Without the wrap, a 350° error and a −10° error look totally different
  //           to the controller even though they are the same physical rotation.
  e = pid_setpoint.yaw - gyro.gz_dps;   // e = setpoint − actual
  if      (e >  180) e -= 360;          // wrap: take the short way around
  else if (e < -179) e += 360;

  //  Step 2 — Proportional: how far off we are drives how hard we correct.
  P = PID.p_gain_yaw * e;               // P = Kp * e

  //  Step 3
  if (integrate) PID_mem.i_mem_yaw += PID.i_gain_yaw * e;
  PID_mem.i_mem_yaw = constrain(PID_mem.i_mem_yaw, -max_yaw, max_yaw);
  I = PID_mem.i_mem_yaw;

  //  Step 4
  D = PID.d_gain_yaw * (e - PID_mem.last_yaw_d_error);
  PID_mem.last_yaw_d_error = e;

  //  Step 5
  PID_out.yaw = P + I + D;
  PID_out.yaw = constrain(PID_out.yaw, -max_yaw, max_yaw);

  // ── DEADBAND ──────────────────────────────────────────────────────────
  //  Outputs below 1 deg/s are indistinguishable from sensor noise.
  //  Zero them out so tiny corrections don't cause constant motor jitter.
  if (fabs(PID_out.roll)  < 1.0f) PID_out.roll  = 0.0f;
  if (fabs(PID_out.pitch) < 1.0f) PID_out.pitch = 0.0f;
  if (fabs(PID_out.yaw)   < 1.0f) PID_out.yaw   = 0.0f;

  pid_mem    = PID_mem;
  pid_output = PID_out;
}

// ============================================================
//  PID::reset()
//
//  Clears integral accumulator (i_mem) and derivative memory
//  (last_*_d_error) for all axes.
//
//  Called when motors disarm so stale accumulated error doesn't
//  cause a sudden output spike on the next arm.
// ============================================================
void PID::reset()
{
  pid_mem    = {};
  pid_output = {};
}