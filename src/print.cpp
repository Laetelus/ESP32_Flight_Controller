#include "Flight_Controller.h"
void Flight_Controller::print()
{
    // We don't use this data for our calculations
    ax_mps2 = ((float)acc_x / 16384.0) * 9.81;
    ay_mps2 = ((float)acc_y / 16384.0) * 9.81;
    az_mps2 = ((float)acc_z / 16384.0) * 9.81;

    Serial.print("--------------------");
    Serial.println();
    Serial.printf("Raw Gyro Pitch: %d \n", gyro_pitch);
    Serial.printf("Raw Gyro Roll: %d \n", gyro_roll);
    Serial.printf("Raw Gyro Yaw: %d \n", gyro_yaw);

    Serial.print("--------------------");
    Serial.println();
    Serial.printf("Raw Acc X: %d \n ", acc_x);
    Serial.printf("Raw Acc Y: %d \n ", acc_y);
    Serial.printf("Raw Acc Z: %d \n ", acc_z);

    Serial.print("--------------------");
    Serial.println();
    Serial.printf("Acc X (m/s^2): %.2f \n", ax_mps2);
    Serial.printf("Acc Y (m/s^2): %.2f \n", ay_mps2);
    Serial.printf("Acc Z (m/s^2): %.2f \n", az_mps2);

    Serial.print("--------------------");
    Serial.println();
    Serial.printf("Angle Pitch: %.2f \n", angle_pitch);
    Serial.printf("Angle Roll: %.2f \n", angle_roll);

    Serial.print("--------------------");
    Serial.println();
    Serial.printf("angle_pitch_acc: %.2f \n", angle_pitch_acc);
    Serial.printf("angle_roll_acc: %.2f \n", angle_roll_acc);

    Serial.print("--------------------");
    Serial.println();
    Serial.printf("Pitch Adjust: %.2f \n", pitch_level_adjust);
    Serial.printf("Roll Adjust: %.2f \n", roll_level_adjust);

    Serial.print("--------------------");
    Serial.println();
    Serial.printf("gyro_roll_input: %.2f \n", gyro_roll_input);
    Serial.printf("gyro_pitch_input: %.2f \n", gyro_pitch_input);
    Serial.printf("gyro_yaw_input: %.2f \n", gyro_yaw_input);

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

    // PID Values
    Serial.print("--------------------");
    Serial.println();
    Serial.printf("pid_p_gain_roll: %.2f \n", pid_p_gain_roll);
    Serial.printf("pid_i_gain_roll: %.2f \n", pid_i_gain_roll);
    Serial.printf("pid_d_gain_roll: %.2f \n", pid_d_gain_roll);
    Serial.println();
    Serial.printf("pid_p_gain_pitch: %.2f \n", pid_p_gain_pitch);
    Serial.printf("pid_i_gain_pitch: %.2f \n", pid_i_gain_pitch);
    Serial.printf("pid_d_gain_pitch: %.2f \n", pid_d_gain_pitch);
    Serial.println();
    Serial.printf("pid_p_gain_yaw: %.2f \n", pid_p_gain_yaw);
    Serial.printf("pid_i_gain_yaw: %.2f \n", pid_i_gain_yaw);
    Serial.printf("pid_d_gain_yaw: %.2f \n", pid_d_gain_yaw);

    // Serial.println();
    // // Serial.printf("Start: %d \n", start);
    // Serial.printf("Throttle: %d \n", receiver_input_channel_1);
    // Serial.printf("Yaw:  %d \n", receiver_input_channel_4);
    // Serial.printf("Roll:  %d \n", receiver_input_channel_2);
    // Serial.printf("Pitch:  %d \n", receiver_input_channel_3);

    // Serial.printf("ESC_1:  %d \n", esc_1);
    // Serial.printf("ESC_2: %d \n ", esc_2);
    // Serial.printf("ESC_3: %d \n ", esc_3);
    // Serial.printf("ESC_4: %d \n ", esc_4);
}