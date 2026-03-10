
#include "Flight_Controller.h"
#include "ReceiverInput.h"

void FC::Initialize_ESCs()
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

void FC::mix_motors()
{
  int local_throttle;

  noInterrupts();
  local_throttle = throttlePulseWidth; // Use local copy for throttle signal
  interrupts();

  if (start == 2)
  {
                                                             // The motors are started.
    local_throttle = constrain(local_throttle, 1000, 1800); // Constrain and allow room for control at full throttle

    esc_1 = computeESCValue(local_throttle, -pid_output_pitch, -pid_output_roll, pid_output_yaw); // FR/CCW
    esc_2 = computeESCValue(local_throttle, -pid_output_pitch, pid_output_roll, -pid_output_yaw); // FL/CW
    esc_3 = computeESCValue(local_throttle, pid_output_pitch, -pid_output_roll, -pid_output_yaw); // BR/CW
    esc_4 = computeESCValue(local_throttle, pid_output_pitch, pid_output_roll, pid_output_yaw);   // BL/CCW

    // // Current mixing algorithm matches my oriantation but
    // // Adjusted mixing algorithm for correct motor responses
    // // Yaw seems to be incorrect 
    // esc_1 = computeESCValue(local_throttle, -pid_output_pitch, -pid_output_roll, pid_output_yaw); // FR/CCW
    // esc_2 = computeESCValue(local_throttle, -pid_output_pitch, pid_output_roll, -pid_output_yaw); // FL/CW
    // esc_3 = computeESCValue(local_throttle, pid_output_pitch, -pid_output_roll, pid_output_yaw);  // BR/CW
    // esc_4 = computeESCValue(local_throttle, pid_output_pitch, pid_output_roll, -pid_output_yaw);  // BL/CCW

  }
  else
  {
    // If start is not 2, keep a 1000us pulse for all ESCs
    esc_1 = esc_2 = esc_3 = esc_4 = 1000;
  }

  write_motors(); 

}

void FC::motorControls()
{
  int roll, pitch, throttle, yaw;
  noInterrupts();
  throttle = throttlePulseWidth;   // Throttle
  yaw = yawPulseWidth;        // Yaw
  roll = rollPulseWidth;       // Roll
  pitch = pitchPulseWidth;      // Pitch
  interrupts();


  unsigned long currentTime = millis();

  // Start condition (start = 1)
  if (throttle < 1065 && yaw < 1050)
  {
    if (!isDebounceConditionMet)
    {
      lastDebounceTime = currentTime;
      isDebounceConditionMet = true;
    }
    else if ((currentTime - lastDebounceTime) > debounceDelay && start == 0)
    {
      start = 1;
      isDebounceConditionMet = false; // Reset for next condition
    }
  }

  // Transition to start = 2 (running), handled inside startInitializationSequence
  if (start == 1 && throttle < 1550 && yaw > 1450)
  {
    if (!isDebounceConditionMet)
    {
      lastDebounceTime = currentTime;
      isDebounceConditionMet = true;
    }
    else if ((currentTime - lastDebounceTime) > debounceDelay)
    {
      Reset_PID();
      isDebounceConditionMet = false; // Reset for next condition
    }
  }

  // Turn off motors (start = 0)
  if (start == 2 && throttle <= 1064 && yaw > 1976)
  {
    if (!isDebounceConditionMet)
    {
      lastDebounceTime = currentTime;
      isDebounceConditionMet = true;
    }
    else if ((currentTime - lastDebounceTime) > debounceDelay)
    {
      start = 0;                      // Turn off
      isDebounceConditionMet = false; // Reset for next condition
    }
  }

  // Reset debounce condition if none of the above conditions are met
  if (!(throttle < 1065 && yaw < 1050) &&
      !(start == 1 && throttle < 1550 && yaw > 1450) &&
      !(start == 2 && throttle <= 1064 && yaw > 1976))
  {
    isDebounceConditionMet = false;
  }

  // Perform level flight and PID calculations only if the motors are started
  if (start == 2)
  {
    computeControlSetpoints(roll,pitch,throttle,yaw);
    calculate_pid();
  }
}

int FC::computeESCValue(int throttle, int pitch, int roll, int yaw) {
  int v = throttle + pitch + roll + yaw;
  return constrain(v, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
}


void FC::write_motors(){

  esc1.writeMicroseconds(esc_1); // FR/CCW
  esc2.writeMicroseconds(esc_2); // FL/CW
  esc3.writeMicroseconds(esc_3); // BR/CW
  esc4.writeMicroseconds(esc_4); // BL/CCW
}
