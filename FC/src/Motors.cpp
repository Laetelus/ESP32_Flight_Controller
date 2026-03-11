
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
  esc1.attach(esc_pin1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // FR (Front Right)
  esc2.attach(esc_pin2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // BR (Back Right)
  esc3.attach(esc_pin3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // FL (Front Left)
  esc4.attach(esc_pin4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // BL (Back Left)

  // Normal arm procedure, setting to minimum throttle
  esc_1 = 1000;
  esc_2 = 1000;
  esc_3 = 1000;
  esc_4 = 1000;
  
  //Write the initial values to motors
  write_motors();
}

void Motors::mix_motors(int throttleInput, const PIDOut& pidOutput)
{
    int throttle = throttleInput;
                                                    
    throttle = constrain(throttle, 1000, 1800); // Constrain and allow room for control at full throttle

    esc_1 = computeESCValue(throttle, -pidOutput.pitch, -pidOutput.roll, pidOutput.yaw); // FR/CCW
    esc_2 = computeESCValue(throttle, -pidOutput.pitch, pidOutput.roll, -pidOutput.yaw); // FL/CW
    esc_3 = computeESCValue(throttle, pidOutput.pitch, -pidOutput.roll, -pidOutput.yaw); // BR/CW
    esc_4 = computeESCValue(throttle, pidOutput.pitch, pidOutput.roll, pidOutput.yaw);   // BL/CCW

    // // Current mixing algorithm matches my oriantation but
    // // Adjusted mixing algorithm for correct motor responses
    // // Yaw seems to be incorrect 
    // esc_1 = computeESCValue(local_throttle, -pid_output_pitch, -pid_output_roll, pid_output_yaw); // FR/CCW
    // esc_2 = computeESCValue(local_throttle, -pid_output_pitch, pid_output_roll, -pid_output_yaw); // FL/CW
    // esc_3 = computeESCValue(local_throttle, pid_output_pitch, -pid_output_roll, pid_output_yaw);  // BR/CW
    // esc_4 = computeESCValue(local_throttle, pid_output_pitch, pid_output_roll, -pid_output_yaw);  // BL/CCW

}

int Motors::computeESCValue(int throttle, int pitch, int roll, int yaw) {
  int v = throttle + pitch + roll + yaw;
  return constrain(v, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
}


void Motors::write_motors()
{
  esc1.writeMicroseconds(esc_1); // FR/CCW
  esc2.writeMicroseconds(esc_2); // FL/CW
  esc3.writeMicroseconds(esc_3); // BR/CW
  esc4.writeMicroseconds(esc_4); // BL/CCW

  // // For debugging: print the ESC values
  // Serial.print("ESC Values - FR: "); Serial.print(esc_1);
  // Serial.print(" | FL: "); Serial.print(esc_2);
  // Serial.print(" | BR: "); Serial.print(esc_3);
  // Serial.print(" | BL: "); Serial.println(esc_4);
}


