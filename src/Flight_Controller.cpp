#include <Arduino.h>
#include <ESP32Servo.h> 
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>

/*
TODO: correct inaccurate gyro reading when stationary 
TODO: Test CW and CCW rotation of propellers 
TODO: check if auto-level functionality works properly
TODO: update loop timer. Remove any delay() functions 
TODO: Adjust pitch,roll, and yaw set-points  
TODO: Interrupt states for receiver. delays won't work here 
TODO: Can no longer use PPM. Switching to PWM 
*/

//
//  FR             BR
//     \           /
//      \---------/
//      |          >      This arrow indicates this is the back side of the quadcopter 
//      /---------\        which is also the battery side of the drone.
//     /           \ 
//  FL             BL

//Might need to remove later if no use 
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs

//Controller 
#define THROTTLE 36 
#define YAW 39  // rudder
#define PITCH 34 // elevator
#define ROLL 35 // aileron

//ESC
#define esc_pin1 32 // FR/CCW 
#define esc_pin2 33 // FL/CW
#define esc_pin3 25 // BR/CW 
#define esc_pin4 26 // BL/CCW 

Servo esc1, esc2, esc3, esc4; 

#define PRINTLN(var) Serial.print(#var ": "); Serial.println(var);

void calculate_pid(); 
void readGyroData(); 
void calibrateMPU650(); 


float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller.
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. 
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int esc_1, esc_2, esc_3, esc_4;
int throttle;
int start;   
float roll_level_adjust, pitch_level_adjust;

int16_t acc_x, acc_y, acc_z, acc_total_vector;
unsigned long loop_timer;
int16_t gyro_pitch, gyro_roll, gyro_yaw;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;

bool auto_level = true; 

MPU6050 accelgyro; 

Servo motA,motB,motC,motD; 

void setup() {

  Serial.begin(115200);
  Wire.begin();
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  // verify connection
  #ifdef I2CDEV_IMPLEMENTATION
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "accelgyro6050 connection successful" : "accelgyro6050 connection failed");
  #endif
  
  Wire.begin();                                                             //Start the I2C as master.
  
  pinMode(2,OUTPUT); //LED status 

  calibrateMPU650();
  //accelgyro.CalibrateGyro();

  
  //attach esc pins 
  esc1.attach(esc_pin1,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH);
  esc2.attach(esc_pin2,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH);
  esc3.attach(esc_pin3,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH);
  esc4.attach(esc_pin4,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH);

  //arm ecs
  esc1.writeMicroseconds(MIN_PULSE_LENGTH);
  esc2.writeMicroseconds(MIN_PULSE_LENGTH);
  esc3.writeMicroseconds(MIN_PULSE_LENGTH); 
  esc4.writeMicroseconds(MIN_PULSE_LENGTH);

  pinMode(THROTTLE, INPUT);
  pinMode(YAW, INPUT);
  pinMode(PITCH, INPUT);
  pinMode(ROLL, INPUT);

}

void loop() {


  //Remove PPM here, PWM will be replaced 
  //Read the raw channel values
  int16_t receiver_input_channel_3 = pulseIn(THROTTLE, HIGH, 25000); //throttle 
  int16_t receiver_input_channel_4 = pulseIn(YAW, HIGH, 25000); //Yaw
  int16_t receiver_input_channel_1 = pulseIn(ROLL, HIGH, 25000); //roll
  int16_t receiver_input_channel_2 = pulseIn(PITCH, HIGH, 25000); //PITCH

  Serial.print("\nTHROTTLE: "); Serial.println(receiver_input_channel_3); 
  Serial.print("YAW: "); Serial.println(receiver_input_channel_4); 
  Serial.print("ROLL: "); Serial.println(receiver_input_channel_1);
  Serial.print("PITCH: "); Serial.println(receiver_input_channel_2);
  
  // // Read accelerometer values.
  // int16_t acc_x = accelgyro.getAccelerationX();
  // int16_t acc_y = accelgyro.getAccelerationY();
  // int16_t acc_z = accelgyro.getAccelerationZ();
  
  readGyroData(); 
  
 //65.5 = 1 deg/sec (check the datasheet of the accelgyro-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.


  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.



  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
  
  if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
  }
  if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }
  
  //Place the accelgyro-6050 spirit level and note the values in the following two lines for calibration.
  angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.
  
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction

  if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
  }

  // Serial.println();
  // PRINTLN(pitch_level_adjust);
  // PRINTLN(roll_level_adjust);   
  // PRINTLN(gyro_yaw);

   //For starting the motors: throttle low and yaw left (step 1).
  if(receiver_input_channel_3 < 1065 && receiver_input_channel_4 < 1050)
  {
    start = 1;
  }
  

  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && receiver_input_channel_3 < 1550 && receiver_input_channel_4 > 1450)
  {
    start = 2;

    angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    gyro_angles_set = true;                                                 //Set the IMU started flag.

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  
  //Stopping the motors: throttle low and yaw right.
  if(start == 2 && receiver_input_channel_3 <= 1064 && receiver_input_channel_4 > 1976)
  {
    start = 0;
  }

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_1 > 1508){
    pid_roll_setpoint = receiver_input_channel_1 - 1508;
  }
  else if(receiver_input_channel_1 < 1492){
    pid_roll_setpoint = receiver_input_channel_1 - 1492;
  }

  pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_2 > 1508){
    pid_pitch_setpoint = receiver_input_channel_2 - 1508;
  }
  else if(receiver_input_channel_2 < 1492){
    pid_pitch_setpoint = receiver_input_channel_2 - 1492;
  }

  pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of dividing by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
    if(receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
    else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
  }
  
  calculate_pid();                                                            //PID inputs are known. So we can calculate the pid output.

  throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.
  if (start == 2)
  {                                                          //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    
    //escs are motor motA,motB,motC,motD 
    esc_1 = map(throttle + pid_output_pitch - pid_output_roll + pid_output_yaw,1000,2000,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH); // FR
    esc_2 = map(throttle - pid_output_pitch - pid_output_roll - pid_output_yaw,1000,2000,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH); // BR
    esc_3 = map(throttle + pid_output_pitch + pid_output_roll - pid_output_yaw,1000,2000,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH); // FL
    esc_4 = map(throttle - pid_output_pitch + pid_output_roll + pid_output_yaw,1000,2000,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH); // BL 
    
    if (esc_1 < 1100) esc_1 = 1100;     //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;     //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;     //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;     //Keep the motors running.

    if(esc_1 > 2000)esc_1 = 2000;       //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;       //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;       //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;       //Limit the esc-4 pulse to 2000us.  
  }

  else
  {
    esc_1 = 1000;                       //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                       //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                       //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                       //If start is not 2 keep a 1000us pulse for ess-4.
  }

    esc1.writeMicroseconds(esc_1); // CCW
    esc2.writeMicroseconds(esc_2); // CW
    esc3.writeMicroseconds(esc_3); // CW
    esc4.writeMicroseconds(esc_4); // CCW

  Serial.println();
  // PRINTLN(esc_1);
  // PRINTLN(esc_2);   
  // PRINTLN(esc_3);
  // PRINTLN(esc_4);
    
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important. If the loop time is 
  //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure 
  //that the loop time is still 4000us and no longer! More information can be found on 
  //the Q&A page: 
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

  // while (micros() - loop_timer < 4000);                                            //We wait until 4000us are passed.
  // loop_timer = micros();                                                           //Set the timer for the next loop.

  //uncomment for debugging
  delay(500);
}

void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void readGyroData() {
  // Read gyroscope values
accelgyro.getMotion6(&acc_x, &acc_y, &acc_z, &gyro_roll, &gyro_pitch, &gyro_yaw);
}


void calibrateMPU650() {
  // Assuming you need to calibrate gyroX, gyroY, and gyroZ
  const int calibrationSamples = 2000; // Number of samples to collect for calibration
  float gyroXOffset = 0.0;
  float gyroYOffset = 0.0;
  float gyroZOffset = 0.0;
  
  // Collect samples and calculate the average
  for (int i = 0; i < calibrationSamples; i++) {
    
    // Read gyro data
    float gyroX = accelgyro.getRotationX();
    float gyroY = accelgyro.getRotationY();
    float gyroZ = accelgyro.getRotationZ();

    // Accumulate offsets
    gyroXOffset += gyroX;
    gyroYOffset += gyroY;
    gyroZOffset += gyroZ; 
    digitalWrite(2,HIGH);  //Calibration indication 

    while (micros() - loop_timer < 5000);                                            //We wait until 5000us are passed.
      loop_timer = micros();                                                         //Set the timer for the next loop.

  }

  // Calculate the average offsets
  gyroXOffset /= calibrationSamples;
  gyroYOffset /= calibrationSamples;
  gyroZOffset /= calibrationSamples;
  
  // Set the gyro offsets
  accelgyro.setXGyroOffset(gyroXOffset);
  accelgyro.setYGyroOffset(gyroYOffset);
  accelgyro.setZGyroOffset(gyroZOffset);
  
  digitalWrite(2, LOW); //turn off led indicating calibration completed 
}