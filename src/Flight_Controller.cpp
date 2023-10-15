#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>

/*
TO̶D̶O̶:̶ c̶o̶r̶r̶e̶c̶t̶ i̶n̶a̶c̶c̶u̶r̶a̶t̶e̶ g̶y̶r̶o̶ r̶e̶a̶d̶i̶n̶g̶ w̶h̶e̶n̶ s̶t̶a̶t̶i̶o̶n̶a̶r̶y̶.̶ 
TODO: Test CW and CCW rotation of propellers 
TODO: check if auto-level functionality works properly
TODO: update loop timer.
TODO: Store gyro and accel values to EEPROM. 
NOTE: if we write gyro and Accel values to EEPROM
we need to call setgyrooffset instead of 
subracting the offsets functions because 
this will be done once. 
*/

//
//  FR             BR     
//     \           /
//      \---------/
//      |             <-- back side of the quadcopter 
//      /---------\       which is also the battery side of the drone.
//     /           \ 
//  FL             BL


#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs

// Controller
#define THROTTLE 36
#define YAW 39   // rudder
#define PITCH 34 // elevator
#define ROLL 35  // aileron

// ESC
#define esc_pin1 32 // FR/CCW
#define esc_pin2 33 // FL/CW
#define esc_pin3 25 // BR/CW
#define esc_pin4 26 // BL/CCW

#define PRINTLN(var)       \
  Serial.print(#var ": "); \
  Serial.println(var);

void calculate_pid(); 
void readGyroData(); 
void calibrateMPU6050(); 

float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll; // Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll; // Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll; // Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;         // Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;  // Gain setting for the pitch P-controller.
float pid_i_gain_yaw = 0.02; // Gain setting for the pitch I-controller.
float pid_d_gain_yaw = 0.0;  // Gain setting for the pitch D-controller.
int pid_max_yaw = 400;       // Maximum output of the PID-controller (+/-)

int esc_1, esc_2, esc_3, esc_4;
int throttle;
int start;
float roll_level_adjust, pitch_level_adjust;

int16_t acc_x, acc_y, acc_z, acc_total_vector;
int16_t gyro_pitch, gyro_roll, gyro_yaw;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float gyroXOffset = 0.0 , gyroYOffset = 0.0, gyroZOffset = 0.0,accXOffset = 0.0,  accYOffset = 0.0, accZOffset = 0.0;
boolean gyro_angles_set;
boolean auto_level = true;

// Initialize MPU6050 with I2C address 0x68 (AD0 high)
MPU6050 accelgyro(0x68);

//Servo motors 
Servo esc1, esc2, esc3, esc4;

void setup() {

  Serial.begin(115200);
  pinMode(2,OUTPUT); //LED status 

  // Allow allocation of all timers. Consistent and accurate PWM. 
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
     Fastwire::setup(400, true);
  #endif

  // initialize device. This also sets the gyros and accelerometers 
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  // verify connection
  #ifdef I2CDEV_IMPLEMENTATION
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "accelgyro6050 connection successful" : "accelgyro6050 connection failed");
  #endif     

  // reset offsets
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);                        


  digitalWrite(2,HIGH); // Calibration indicator 
  calibrateMPU6050(); 
  delay(2000); // Lets give it a 2 sec delay
  digitalWrite(2,LOW); //Calibration indicator 
  
  // attach esc pins
  esc1.attach(esc_pin1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // FR (Front Right)
  esc2.attach(esc_pin3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // BR (Back Right)
  esc3.attach(esc_pin2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // FL (Front Left)
  esc4.attach(esc_pin4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // BL (Back Left)

  // arm ecs
  esc1.writeMicroseconds(MIN_PULSE_LENGTH);
  esc2.writeMicroseconds(MIN_PULSE_LENGTH);
  esc3.writeMicroseconds(MIN_PULSE_LENGTH);
  esc4.writeMicroseconds(MIN_PULSE_LENGTH);

  pinMode(THROTTLE, INPUT);
  pinMode(YAW, INPUT);
  pinMode(PITCH, INPUT);
  pinMode(ROLL, INPUT);
}

void loop()
{
  //Read the raw channel values                                                                                                            
  volatile int16_t receiver_input_channel_3 = pulseIn(THROTTLE, HIGH, 25000); //throttle 
  volatile int16_t receiver_input_channel_4 = pulseIn(YAW, HIGH, 25000); //Yaw
  volatile int16_t receiver_input_channel_1 = pulseIn(ROLL, HIGH, 25000); //roll
  volatile int16_t receiver_input_channel_2 = pulseIn(PITCH, HIGH, 25000); //PITCH

  // 65.5 = 1 deg/sec (check the datasheet of the accelgyro-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);    // Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3); // Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);       // Gyro pid input is deg/sec.

  // Gyro angle calculations
  // 0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611; // Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0000611;   // Calculate the traveled roll angle and add this to the angle_roll variable.

  // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066); // If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066); // If the IMU has yawed transfer the pitch angle to the roll angel.

  // Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); // Calculate the total accelerometer vector.

  if (abs(acc_y) < acc_total_vector)
  {                                                                   // Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296; // Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector)
  {                                                                   // Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296; // Calculate the roll angle.
  }

  // Place the accelgyro-6050 spirit level and note the values in the following two lines for calibration.
  angle_pitch_acc -= 0.0; // Accelerometer calibration value for pitch.
  angle_roll_acc -= 0.0;  // Accelerometer calibration value for roll.

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004; // Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;    // Correct the drift of the gyro roll angle with the accelerometer roll angle.

 
  pitch_level_adjust = angle_pitch * 15; // Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;   // Calculate the roll angle correction
  
  
  if (!auto_level)
  {                         // If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0; // Set the pitch angle correction to zero.
    roll_level_adjust = 0;  // Set the roll angle correcion to zero.
  }

  // For starting the motors: throttle low and yaw left (step 1).
  if (receiver_input_channel_3 < 1065 && receiver_input_channel_4 < 1050)
  {
    start = 1;
  }

  // When yaw stick is back in the center position start the motors (step 2).
  if (start == 1 && receiver_input_channel_3 < 1550 && receiver_input_channel_4 > 1450)
  {
    start = 2;

    angle_pitch = angle_pitch_acc; // Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;   // Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    
    gyro_angles_set = true;        // Set the IMU started flag.

    // Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }

  // Stopping the motors: throttle low and yaw right.
  if (start == 2 && receiver_input_channel_3 <= 1064 && receiver_input_channel_4 > 1976)
  {
    start = 0;
  }

  // The PID set point in degrees per second is determined by the roll receiver input.
  // In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;

  // We need a little dead band of 16us for better results.
  if (receiver_input_channel_1 > 1508)
  {
    pid_roll_setpoint = receiver_input_channel_1 - 1508;
  }
  else if (receiver_input_channel_1 < 1492)
  {
    pid_roll_setpoint = receiver_input_channel_1 - 1492;
  }

  pid_roll_setpoint -= roll_level_adjust; // Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;               // Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

  // The PID set point in degrees per second is determined by the pitch receiver input.
  // In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;

  // We need a little dead band of 16us for better results.
  if (receiver_input_channel_2 > 1508)
  {
    pid_pitch_setpoint = receiver_input_channel_2 - 1508;
  }
  else if (receiver_input_channel_2 < 1492)
  {
    pid_pitch_setpoint = receiver_input_channel_2 - 1492;
  }

  pid_pitch_setpoint -= pitch_level_adjust; // Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                // Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  // The PID set point in degrees per second is determined by the yaw receiver input.
  // In the case of dividing by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;

  // We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050)
  { // Do not yaw when turning off the motors.
    if (receiver_input_channel_4 > 1508)
      pid_yaw_setpoint = (receiver_input_channel_4 - 1508) / 3.0;
    else if (receiver_input_channel_4 < 1492)
      pid_yaw_setpoint = (receiver_input_channel_4 - 1492) / 3.0;
  }

  calculate_pid(); // PID inputs are known. So we can calculate the pid output.

  throttle = receiver_input_channel_3; // We need the throttle signal as a base signal.
  if (start == 2)
  { // The motors are started.
    if (throttle > 1800)
      throttle = 1800; // We need some room to keep full control at full throttle.

    // Mixing algorithm for appropriate motors 
    esc_1 = constrain(map(throttle - pid_output_pitch + pid_output_roll - pid_output_yaw, 1000, 2000, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH), MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // FR (Front Right)
    esc_2 = constrain(map(throttle + pid_output_pitch + pid_output_roll + pid_output_yaw, 1000, 2000, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH), MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // BR (Back Right)
    esc_3 = constrain(map(throttle - pid_output_pitch - pid_output_roll + pid_output_yaw, 1000, 2000, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH), MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // FL (Front Left)
    esc_4 = constrain(map(throttle + pid_output_pitch - pid_output_roll - pid_output_yaw, 1000, 2000, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH), MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // BL (Back Left)

    if (esc_1 < 1100)
      esc_1 = 1100; // Keep the motors running.
    if (esc_2 < 1100)
      esc_2 = 1100; // Keep the motors running.
    if (esc_3 < 1100)
      esc_3 = 1100; // Keep the motors running.
    if (esc_4 < 1100)
      esc_4 = 1100; // Keep the motors running.

    if (esc_1 > 2000)
      esc_1 = 2000; // Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000)
      esc_2 = 2000; // Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000)
      esc_3 = 2000; // Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000)
      esc_4 = 2000; // Limit the esc-4 pulse to 2000us.
  }

  else
  {
    esc_1 = 1000; // If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000; // If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000; // If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000; // If start is not 2 keep a 1000us pulse for ess-4.
  }

  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  // Because of the angle calculation the loop time is getting very important. If the loop time is
  // longer or shorter than 4000us the angle calculation is off. If you modify the code make sure
  // that the loop time is still 4000us and no longer! 
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

  delayMicroseconds(4000); 

  // There is always 1000us of spare time. So let's do something useful that is very time consuming.
  // Get the current gyro and receiver data and scale it to degrees per second for the pid calculations.
  readGyroData();

  /*
    whenever we retrieve gyro and accelerometer data, 
    we'll want to subtract these offsets from the raw values
  */
  gyro_roll -= gyroXOffset;
  gyro_pitch -= gyroYOffset;
  gyro_yaw -= gyroZOffset;

  acc_x -= accXOffset;
  acc_y -= accYOffset;
  acc_z -= accZOffset;
  
  esc1.writeMicroseconds(esc_1);
  esc2.writeMicroseconds(esc_2);
  esc3.writeMicroseconds(esc_3);
  esc4.writeMicroseconds(esc_4);

  Serial.print("Roll: ");
  Serial.print((float)gyro_roll / 131);
  Serial.print(", ");
  Serial.print("Pitch: ");
  Serial.print((float)gyro_pitch / 131);
  Serial.print(", ");
    Serial.print("Yaw: ");
  Serial.println((float)gyro_yaw / 131);  
}

void calculate_pid()
{
  // Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)
    pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)
    pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)
    pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)
    pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  // Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)
    pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)
    pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)
    pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)
    pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  // Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)
    pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)
    pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)
    pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)
    pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void readGyroData()
{
  // Read Raw gyroscope/accelerometer values
  accelgyro.getMotion6(&acc_x, &acc_y, &acc_z, &gyro_roll, &gyro_pitch, &gyro_yaw);
}

/*
After calibration, the gyro offsets should help ensure 
that the gyro readings are about zero or close to zero 
when the IMU is stationary, which is the desired behavior 
for accurate sensor data
*/
void calibrateMPU6050() {
  const int calibrationSamples = 1000;
  const int discardSamples = 100;
  const int acel_deadzone = 8;
  const int giro_deadzone = 1;
  const int max_iterations = 10; // Maximum number of iterative calibration rounds
  
  long buff_ax, buff_ay, buff_az, buff_gx, buff_gy, buff_gz;

  for (int iteration = 0; iteration < max_iterations; iteration++) {
    buff_ax = buff_ay = buff_az = buff_gx = buff_gy = buff_gz = 0;

    for (int i = 0; i < calibrationSamples + discardSamples; i++) {
      readGyroData(); 
      if (i >= discardSamples) { // Discard the first "discardSamples" readings
        buff_ax += acc_x;
        buff_ay += acc_y;
        buff_az += acc_z;
        buff_gx += gyro_roll;
        buff_gy += gyro_pitch;
        buff_gz += gyro_yaw;
      }
      delay(2); 
    }

    // Calculate the average offsets
    int mean_ax = buff_ax / calibrationSamples;
    int mean_ay = buff_ay / calibrationSamples;
    int mean_az = buff_az / calibrationSamples;
    int mean_gx = buff_gx / calibrationSamples;
    int mean_gy = buff_gy / calibrationSamples;
    int mean_gz = buff_gz / calibrationSamples;

    // Start calibration integration
    int ready = 0;

    accXOffset = -mean_ax / 8.0;
    accYOffset = -mean_ay / 8.0;
    accZOffset = (16384 - mean_az) / 8.0;
    gyroXOffset = -mean_gx / 4.0;
    gyroYOffset = -mean_gy / 4.0;
    gyroZOffset = -mean_gz / 4.0;

    accelgyro.setXAccelOffset(accXOffset);
    accelgyro.setYAccelOffset(accYOffset);
    accelgyro.setZAccelOffset(accZOffset);
    accelgyro.setXGyroOffset(gyroXOffset);
    accelgyro.setYGyroOffset(gyroYOffset);
    accelgyro.setZGyroOffset(gyroZOffset);

    if (abs(mean_ax) <= acel_deadzone) ready++;
    else accXOffset -= mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone) ready++;
    else accYOffset -= mean_ay / acel_deadzone;

    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else accZOffset += (16384 - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone) ready++;
    else gyroXOffset -= mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone) ready++;
    else gyroYOffset -= mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone) ready++;
    else gyroZOffset -= mean_gz / (giro_deadzone + 1);

    if (ready == 6) break;
  }
}





