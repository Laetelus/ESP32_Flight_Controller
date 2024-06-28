#include "Flight_Controller.h"

void Flight_Controller::print()
{

  // Serial.print("--------------------");
  // Serial.println();
  // Serial.print("Raw Gyro Pitch: ");
  // Serial.println(raw_gy);
  // Serial.print("Raw Gyro Roll: ");
  // Serial.println(raw_gx);
  // Serial.print("Raw Gyro Yaw: ");
  // Serial.println(raw_gz);

  // Serial.print("--------------------");
  // Serial.println();
  // Serial.print("Raw Acc X: ");
  // Serial.println(raw_ax);
  // Serial.print("Raw Acc Y: ");
  // Serial.println(raw_ay);
  // Serial.print("Raw Acc Z:");
  // Serial.println(raw_az);

  // Serial.print("--------------------");
  // Serial.println();
  // Serial.printf("Acc X (g): %.2f \n", ax_g);
  // Serial.printf("Acc Y (g): %.2f \n", ay_g);
  // Serial.printf("Acc Z (g): %.2f \n", az_g);

  // Serial.print("--------------------");
  // Serial.println();
  // Serial.print("Acc roll in degrees: ");
  // Serial.println(accRoll);
  // Serial.print("Acc pitch in degrees: ");
  // Serial.println(accPitch);

  // Serial.print("--------------------");
  // Serial.println();
  // Serial.printf("Angle Pitch: %.2f \n", angle_pitch);
  // Serial.printf("Angle Roll: %.2f \n", angle_roll);

  // Serial.print("--------------------");
  // Serial.println();
  // Serial.printf("pid_roll_setpoint: %.2f \n", pid_roll_setpoint);
  // Serial.printf("pid_pitch_setpoint: %.2f \n", pid_pitch_setpoint);
  // Serial.printf("pid_yaw_setpoint: %.2f \n", pid_yaw_setpoint);

  // Serial.print("--------------------");
  // Serial.println();
  // Serial.printf("Pitch Adjust: %.2f \n", pitch_level_adjust);
  // Serial.printf("Roll Adjust: %.2f \n", roll_level_adjust);

  Serial.print("--------------------");
  Serial.println();
  Serial.printf("gyro_roll_input°: %.2f \n", gyro_roll_input);
  Serial.printf("gyro_pitch_input°: %.2f \n", gyro_pitch_input);
  Serial.printf("gyro_yaw_input°: %.2f \n", gyro_yaw_input);

  Serial.print("--------------------");
  Serial.println();
  Serial.printf("pid_output_roll: %.2f \n", pid_output_roll);
  Serial.printf("pid_output_pitch: %.2f \n", pid_output_pitch);
  Serial.printf("pid_output_yaw: %.2f \n", pid_output_yaw);

  Serial.print("--------------------");
  Serial.println();
  Serial.printf("pid_last_roll_d_error: %.2f \n", pid_last_roll_d_error);
  Serial.printf("pid_last_pitch_d_error: %.2f \n", pid_last_pitch_d_error);
  Serial.printf("pid_last_yaw_d_error: %.2f \n", pid_last_yaw_d_error);

  // // KP input Values from webserver
  // Serial.print("--------------------");
  // Serial.println();
  // Serial.printf("pid_p_gain_roll: %.2f \n", pid_p_gain_roll);
  // Serial.printf("pid_i_gain_roll: %.2f \n", pid_i_gain_roll);
  // Serial.printf("pid_d_gain_roll: %.2f \n", pid_d_gain_roll);
  // Serial.println();
  // Serial.printf("pid_p_gain_pitch: %.2f \n", pid_p_gain_pitch);
  // Serial.printf("pid_i_gain_pitch: %.2f \n", pid_i_gain_pitch);
  // Serial.printf("pid_d_gain_pitch: %.2f \n", pid_d_gain_pitch);
  // Serial.println();
  // Serial.printf("pid_p_gain_yaw: %.2f \n", pid_p_gain_yaw);
  // Serial.printf("pid_i_gain_yaw: %.2f \n", pid_i_gain_yaw);
  // Serial.printf("pid_d_gain_yaw: %.2f \n", pid_d_gain_yaw);
  // Serial.println();

  // Serial.println();
  // // Serial.printf("Start: %d \n", start);
  // Serial.printf("Throttle: %d \n", receiver_input_channel_1);
  // Serial.printf("Yaw:  %d \n", receiver_input_channel_4);
  // Serial.printf("Roll:  %d \n", receiver_input_channel_2);
  // Serial.printf("Pitch:  %d \n", receiver_input_channel_3);

  // used for checking if mixing algorithm matches and outputs correspond correctly
  // Serial.print("--------------------");
  // Serial.println();
  // Serial.printf("ESC_1: %d \n", esc_1);
  // Serial.printf("ESC_2: %d \n ", esc_2);
  // Serial.printf("ESC_3: %d \n ", esc_3);
  // Serial.printf("ESC_4: %d \n ", esc_4);
}