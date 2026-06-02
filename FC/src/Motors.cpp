
#include "Flight_Controller.h"
#include "ReceiverInput.h"
#include "Motors.h"

void Motors::Initialize_ESCs()
{
  // Allow allocation of all timers. 
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Attach ESC pins
  esc1.attach(esc_pin1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // FR (Front Right) pin25
  esc2.attach(esc_pin2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // BR (Back  Right) pin32  ← physically back-right
  esc3.attach(esc_pin3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // FL (Front Left)  pin26  ← physically front-left
  esc4.attach(esc_pin4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // BL (Back  Left)  pin33

  // Normal arm procedure, setting to minimum throttle
  idle(); 
  //Write the initial values to motors
  write_motors();
}

void Motors::mix_motors(int throttleInput, const PIDOut& pidOutput)
{
    int throttle = constrain(throttleInput, 1000, 1800); // allow room for PID authority at full throttle

    // X-frame mixing. Physical arm layout (verified by bench test):
    //   FR (pin25/CCW)  FL (pin26/CW)
    //   BR (pin32/CW)   BL (pin33/CCW)
    // Pitch +  = nose-down → front motors slow, back motors speed up
    // Roll  +  = right-down → right motors slow, left motors speed up
    // Yaw   +  = CW        → CCW motors (FR,BL) speed up, CW motors (FL,BR) slow
    esc_1 = computeESCValue(throttle, -pidOutput.pitch, -pidOutput.roll,  pidOutput.yaw); // FR/CCW  pin25
    esc_2 = computeESCValue(throttle,  pidOutput.pitch, -pidOutput.roll, -pidOutput.yaw); // BR/CW   pin32
    esc_3 = computeESCValue(throttle, -pidOutput.pitch,  pidOutput.roll, -pidOutput.yaw); // FL/CW   pin26
    esc_4 = computeESCValue(throttle,  pidOutput.pitch,  pidOutput.roll,  pidOutput.yaw); // BL/CCW  pin33

    // Serial.printf("PID  R:%6.1f  P:%6.1f  Y:%6.1f\n", pidOutput.roll, pidOutput.pitch, pidOutput.yaw);
    // Serial.printf("ESC  FR:%4d  FL:%4d  BR:%4d  BL:%4d\n", esc_1, esc_2, esc_3, esc_4);  // use FC::print() instead

}

int Motors::computeESCValue(int throttle, int pitch, int roll, int yaw) {
  int v = throttle + pitch + roll + yaw;
  // Clamp to MOTOR_MIN_SPIN (not MIN_PULSE_LENGTH) so no motor falls below its
  // spin threshold due to PID corrections or ESC deadband mismatch.
  // idle() bypasses this and writes 1000 directly.
  return constrain(v, MOTOR_MIN_SPIN, MAX_PULSE_LENGTH);
}


void Motors::write_motors()
{
  esc1.writeMicroseconds(esc_1); // FR/CCW
  esc2.writeMicroseconds(esc_2); // FL/CW
  esc3.writeMicroseconds(esc_3); // BR/CW
  esc4.writeMicroseconds(esc_4); // BL/CCW

}


