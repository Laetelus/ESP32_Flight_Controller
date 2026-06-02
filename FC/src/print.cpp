#include "Flight_Controller.h"

void FC::print()
{
  // Print axis convention legend once per power-on so every log is self-documenting.
  static bool legendPrinted = false;
  if (!legendPrinted)
  {
    Serial.println(F("\n=== AXIS CONVENTION ==="));
    Serial.println(F("  roll  + = right side DOWN   (tilting right)"));
    Serial.println(F("  roll  - = left  side DOWN   (tilting left)"));
    Serial.println(F("  pitch + = nose  DOWN         (pitching forward)"));
    Serial.println(F("  pitch - = nose  UP           (pitching back)"));
    Serial.println(F("  gx  + = rotating right      (right-hand roll rate)"));
    Serial.println(F("  gy  + = pitching nose-down  (right-hand pitch rate)"));
    Serial.println(F("  gz  + = yawing clockwise     (viewed from above)"));
    Serial.println(F("  AUTO-LEVEL: sp.roll should OPPOSE angle"));
    Serial.println(F("    e.g. roll=+14deg → sp.roll should be NEGATIVE"));
    Serial.println(F("=======================\n"));
    legendPrinted = true;
  }

  const FilteredAttitude&  att    = imu.getAttitude();
  const AccelAngleData&    accel  = imu.getAccelAngles();
  const ScaledImuData&     gyro   = imu.getScaledData();
  const PIDSetpoints       sp     = pid.getSetpoints();
  const PIDOut             out    = pid.getOutput();
  const ESCValues          esc    = motors.getLastESC();

  const char* stateStr = (state == RUNNING) ? "RUN"   :
                         (state == START)   ? "START" : "OFF";

  switch (printMode_)
  {
    // ----------------------------------------------------------------
    // IMU layer — verify the sensor fusion / complementary filter
    // ----------------------------------------------------------------
    case PRINT_IMU:
      Serial.println(F("\n--- IMU ---"));
      // Accel-only angles: long-term correct but noisy from vibration
      Serial.printf("  Accel    roll:%7.2f  pitch:%7.2f  deg\n",
                    accel.Roll, accel.Pitch);
      // Comp-filter angles: gyro integrated + accel correction (what auto-level uses)
      Serial.printf("  CompFilt roll:%7.2f  pitch:%7.2f  deg\n",
                    att.roll_deg, att.pitch_deg);
      // Filtered gyro rates: IIR smoothed, fed directly to PID
      Serial.printf("  Gyro     gx:%7.2f   gy:%7.2f    gz:%7.2f  deg/s\n",
                    gyro.gx_dps, gyro.gy_dps, gyro.gz_dps);
      break;

    // ----------------------------------------------------------------
    // Control layer — verify sticks → setpoints → PID output
    // ----------------------------------------------------------------
    case PRINT_CONTROL:
      Serial.println(F("\n--- CONTROL ---"));
      // Raw receiver µs values — verify sticks are being read
      Serial.printf("  Sticks   thr:%4d  yaw:%4d  roll:%4d  pitch:%4d\n",
                    lastInput_.throttle, lastInput_.yaw,
                    lastInput_.roll,     lastInput_.pitch);
      // PID setpoints in deg/s — stick delta minus auto-level correction, / 3
      // At centre sticks level: setpoints should oppose the tilt angle
      Serial.printf("  Setpts   roll:%7.2f  pitch:%7.2f  yaw:%7.2f  deg/s\n",
                    sp.roll, sp.pitch, sp.yaw);
      // Gyro rates — what the rate PID error is measured against
      Serial.printf("  Gyro     roll:%7.2f  pitch:%7.2f  yaw:%7.2f  deg/s\n",
                    gyro.gx_dps, gyro.gy_dps, gyro.gz_dps);
      // PID output — correction added/subtracted from throttle in mixing
      Serial.printf("  PID out  roll:%7.1f  pitch:%7.1f  yaw:%7.1f\n",
                    out.roll, out.pitch, out.yaw);
      Serial.printf("  State: %s\n", stateStr);
      break;

    // ----------------------------------------------------------------
    // Motor layer — verify mixing for your frame orientation
    // ----------------------------------------------------------------
    case PRINT_MOTORS:
      Serial.println(F("\n--- MOTORS ---"));
      //  FR(CCW) FL(CW)
      //  BR(CW)  BL(CCW)
      Serial.printf("  FR:%4d  FL:%4d\n", esc.fr, esc.fl);
      Serial.printf("  BR:%4d  BL:%4d\n", esc.br, esc.bl);
      Serial.printf("  Angles  roll:%6.1f  pitch:%6.1f  State:%s\n",
                    att.roll_deg, att.pitch_deg, stateStr);
      Serial.printf("  Sticks  thr:%4d  roll:%4d  pitch:%4d  yaw:%4d\n",
                    lastInput_.throttle, lastInput_.roll,
                    lastInput_.pitch,    lastInput_.yaw);
      Serial.printf("  Setpts  roll:%7.2f  pitch:%7.2f  yaw:%7.2f  deg/s\n",
                    sp.roll, sp.pitch, sp.yaw);
      break;

    // ----------------------------------------------------------------
    // CSV — full snapshot for Python/Excel graphing
    // ----------------------------------------------------------------
    case PRINT_CSV:
    {
      static bool hdr = false;
      if (!hdr) {
        Serial.println(F("ms,thr,yaw_stick,roll_stick,pitch_stick,"
                         "acc_roll,acc_pitch,"
                         "filt_roll,filt_pitch,"
                         "gx,gy,gz,"
                         "sp_roll,sp_pitch,sp_yaw,"
                         "pid_roll,pid_pitch,pid_yaw,"
                         "FR,FL,BR,BL,state"));
        hdr = true;
      }
      Serial.printf("%lu,%d,%d,%d,%d,"
                    "%.2f,%.2f,"
                    "%.2f,%.2f,"
                    "%.2f,%.2f,%.2f,"
                    "%.2f,%.2f,%.2f,"
                    "%.1f,%.1f,%.1f,"
                    "%d,%d,%d,%d,%s\n",
        millis(),
        lastInput_.throttle, lastInput_.yaw, lastInput_.roll, lastInput_.pitch,
        accel.Roll,    accel.Pitch,
        att.roll_deg,  att.pitch_deg,
        gyro.gx_dps,   gyro.gy_dps,  gyro.gz_dps,
        sp.roll,       sp.pitch,     sp.yaw,
        out.roll,      out.pitch,    out.yaw,
        esc.fr, esc.fl, esc.br, esc.bl, stateStr);
      break;
    }
  }
}