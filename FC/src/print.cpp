#include "Flight_Controller.h"

// // For python
// void Flight_Controller::print()
// {

//   // Serial.print("--------------------");
//   // Serial.println();
//   // Serial.print("Raw Gyro Pitch: ");
//   // Serial.println(raw_gy);
//   // Serial.print("Raw Gyro Roll: ");
//   // Serial.println(raw_gx);
//   // Serial.print("Raw Gyro Yaw: ");
//   // Serial.println(raw_gz);

//   // Serial.print("--------------------");
//   // Serial.println();
//   // Serial.print("Raw Acc X: ");
//   // Serial.println(raw_ax);
//   // Serial.print("Raw Acc Y: ");
//   // Serial.println(raw_ay);
//   // Serial.print("Raw Acc Z:");
//   // Serial.println(raw_az);

// //   // Serial.print("--------------------");
// //   // Serial.println();
// //   // Serial.printf("Acc X (g): %.2f \n", ax_g);
// //   // Serial.printf("Acc Y (g): %.2f \n", ay_g);
// //   // Serial.printf("Acc Z (g): %.2f \n", az_g);

// //   // Serial.print("--------------------");
// //   // Serial.println();
// //   // Serial.print("Acc roll in °: ");
// //   // Serial.println(accRoll);
// //   // Serial.print("Acc pitch in °: ");
// //   // Serial.println(accPitch);

// //   // Serial.print("--------------------");
// //   // Serial.println();
// //   // Serial.printf("Angle Pitch: %.2f \n", angle_pitch);
// //   // Serial.printf("Angle Roll: %.2f \n", angle_roll);

// //   // Serial.print("--------------------");
// //   // Serial.println();
// //   // Serial.printf("pid_roll_setpoint: %.2f \n", pid_roll_setpoint);
// //   // Serial.printf("pid_pitch_setpoint: %.2f \n", pid_pitch_setpoint);
// //   // Serial.printf("pid_yaw_setpoint: %.2f \n", pid_yaw_setpoint);

// //   // Serial.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
// //   //               pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint,
// //   //               angle_roll, angle_pitch, gyro_yaw_input,
// //   //               pid_output_roll, pid_output_pitch, pid_output_yaw);

// //   Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
// //                 millis(),                              // Correctly using %lu for unsigned long
// //                 pid_roll_setpoint, pid_pitch_setpoint, // Using %.2f for floats
// //                 angle_roll, angle_pitch,               // Using %.2f for floats
// //                 pid_output_roll, pid_output_pitch);    // Using %.2f for floats

// //   // Serial.print("--------------------");
// //   // Serial.println();
// //   // Serial.printf("Pitch Adjust: %.2f \n", pitch_level_adjust);
// //   // Serial.printf("Roll Adjust: %.2f \n", roll_level_adjust);

// //   // Serial.print("--------------------");
// //   // Serial.println();
// //   // Serial.printf("gyro_roll_input°: %.2f \n", gyro_roll_input);
// //   // Serial.printf("gyro_pitch_input°: %.2f \n", gyro_pitch_input);
// //   // Serial.printf("gyro_yaw_input°: %.2f \n", gyro_yaw_input);

// //   // Serial.print("--------------------");
// //   // Serial.println();
// //   // Serial.printf("pid_output_roll: %.2f \n", pid_output_roll);
// //   // Serial.printf("pid_output_pitch: %.2f \n", pid_output_pitch);
// //   // Serial.printf("pid_output_yaw: %.2f \n", pid_output_yaw);

// //   // Serial.print("--------------------");
// //   // Serial.println();
// //   // Serial.printf("pid_last_roll_d_error: %.2f \n", pid_last_roll_d_error);
// //   // Serial.printf("pid_last_pitch_d_error: %.2f \n", pid_last_pitch_d_error);
// //   // Serial.printf("pid_last_yaw_d_error: %.2f \n", pid_last_yaw_d_error);

// //   // // KP input Values from webserver
// //   // Serial.print("--------------------");
// //   // Serial.println();
// //   // Serial.printf("pid_p_gain_roll: %.2f \n", pid_p_gain_roll);
// //   // Serial.printf("pid_i_gain_roll: %.2f \n", pid_i_gain_roll);
// //   // Serial.printf("pid_d_gain_roll: %.2f \n", pid_d_gain_roll);
// //   // Serial.println();
// //   // Serial.printf("pid_p_gain_pitch: %.2f \n", pid_p_gain_pitch);
// //   // Serial.printf("pid_i_gain_pitch: %.2f \n", pid_i_gain_pitch);
// //   // Serial.printf("pid_d_gain_pitch: %.2f \n", pid_d_gain_pitch);
// //   // Serial.println();
// //   // Serial.printf("pid_p_gain_yaw: %.2f \n", pid_p_gain_yaw);
// //   // Serial.printf("pid_i_gain_yaw: %.2f \n", pid_i_gain_yaw);
// //   // Serial.printf("pid_d_gain_yaw: %.2f \n", pid_d_gain_yaw);
// //   // Serial.println();

//   // // // used for checking if mixing algorithm matches and outputs correspond correctly
//   // Serial.print("--------------------");
//   // Serial.println();
//   // Serial.printf("ESC_1: %d \n", esc_1);
//   // Serial.printf("ESC_2: %d \n ", esc_2);
//   // Serial.printf("ESC_3: %d \n ", esc_3);
//   // Serial.printf("ESC_4: %d \n ", esc_4);
// }

// For ESP debugging
  void FC::print()
  {
      // Print data in a comma-separated format for the Serial Plotter
      // Serial.print(millis());               // Timestamp in milliseconds

  // Serial.print("angle_roll:");
  // Serial.print(angle_roll);
  // Serial.print(",");
  // Serial.print("angle_pitch:");
  // Serial.println(angle_pitch);

  // Serial.print("pid_output_roll:");
  // Serial.print(pid_output_roll);
  // Serial.print(",");
  // Serial.print("pid_output_pitch:");
  // Serial.println(pid_output_pitch);
  // Serial.print(",");
  // Serial.print("pid_output_yaw:");
  // Serial.println(pid_output_yaw);

    // Serial.print(",");                    // Separator (comma)
    // Serial.print(pid_roll_setpoint);   // PID Roll Setpoint with 2 decimal places
    // Serial.print(",");
    // Serial.print(pid_pitch_setpoint);  // PID Pitch Setpoint
    // Serial.print(",");
    // Serial.print(angle_roll);          // Roll Angle
    // Serial.print(",");
    // Serial.print(angle_pitch);         // Pitch Angle
    // Serial.print(",");
    // Serial.print(pid_output_roll);     // PID Output Roll
    // Serial.print(",");
    // Serial.println(pid_output_pitch);  // PID Output Pitch (ends the line)
}