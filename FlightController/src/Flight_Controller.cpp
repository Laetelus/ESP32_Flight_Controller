#include <PPMReader.h>
#include <ESP32Servo.h> 
#include "I2Cdev.h"
#include "MPU6050.h"
// Initialize a PPMReader on digital pin 3 with 6 expected channels.
byte interruptPin = 13; // PPM pin connector 
byte channelAmount = 4; // Number of channels to use 

// Define motor pins 
#define MOTOR_1_PIN 14
#define MOTOR_2_PIN 27
#define MOTOR_3_PIN 26
#define MOTOR_4_PIN 25

#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
//#define MID_PULSE_LENGTH 1500 //Neutral pulse length in µs

Servo motA,motB,motC,motD; 
PPMReader ppm(interruptPin, channelAmount);

//Do not exceed kp > 2, ki, > 0.5, kd > 2  
#define PID_PITCH_P   0
#define PID_PITCH_D   0
#define PID_PITCH_I   0

#define PID_ROLL_P    0
#define PID_ROLL_D    0
#define PID_ROLL_I    0

#define PID_YAW_P     0
#define PID_YAW_D     0
#define PID_YAW_I     0

MPU6050 accelgyro; 

int16_t inline ExecutePitchPID(const int16_t& pitch_set_point, const int16_t& measured_pitch);
int16_t inline ExecuteRollPID(const int16_t& roll_set_point, const int16_t& measured_roll);
int16_t inline ExecuteYawPID(const int16_t& yaw_set_point, const int16_t& measured_yaw); 
void inline MeasurePitchRollYaw(int16_t& measured_pitch, int16_t& measured_roll, int16_t& measured_yaw);

void setup() {

  Serial.begin(115200);

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
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  #endif
  // Set up the motors
  motA.attach(MOTOR_1_PIN,MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motB.attach(MOTOR_2_PIN,MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motC.attach(MOTOR_3_PIN,MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motD.attach(MOTOR_4_PIN,MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  //Calibrate the ESCs
  motA.writeMicroseconds(MIN_PULSE_LENGTH); 
  motB.writeMicroseconds(MIN_PULSE_LENGTH); 
  motC.writeMicroseconds(MIN_PULSE_LENGTH); 
  motD.writeMicroseconds(MIN_PULSE_LENGTH); 
}

void loop() {

  //Read the raw channel values
  int16_t THROTTLE = ppm.rawChannelValue(3);
  int16_t YAW = ppm.rawChannelValue(4);
  int16_t ROLL = ppm.rawChannelValue(1);
  int16_t PITCH = ppm.rawChannelValue(2);
  
  int16_t   measured_pitch  = 0;
  int16_t   measured_roll   = 0;
  int16_t   measured_yaw    = 0;

  // Serial.println();
  // Serial.print("THROTTLE: " + String(THROTTLE) + "\n");
  // Serial.print("YAW: " + String(YAW) + "\n");
  // Serial.print("PITCH: " + String(PITCH) + "\n");
  // Serial.print("ROLL: " + String(ROLL) + "\n");

  // Scale the input values to a range of 0 to 100
  THROTTLE = map(THROTTLE, 1000, 2000, 0, 100);
  YAW = map(YAW, 1000, 2000, -80, 80);
  PITCH = map(PITCH, 1000, 2000, -50, 50);
  ROLL = map(ROLL, 1000, 2000, -50, 50);
        
  MeasurePitchRollYaw(measured_pitch, measured_roll, measured_yaw);
  
  int16_t PitchPIDOutput  = ExecutePitchPID(PITCH, measured_pitch);
  int16_t RollPIDOutput   = ExecuteRollPID(ROLL, measured_roll);
  int16_t YawPIDOutput    = ExecuteYawPID(YAW, measured_yaw);

  // Mix the input values to determine the speed and direction of each motor
  int16_t mota = map(THROTTLE + PitchPIDOutput - RollPIDOutput + YawPIDOutput, 0, 100, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); //FR
  int16_t motb = map(THROTTLE - PitchPIDOutput - RollPIDOutput - YawPIDOutput, 0, 100, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); //BR
  int16_t motc = map(THROTTLE + PitchPIDOutput + RollPIDOutput - YawPIDOutput, 0, 100, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); //FL
  int16_t motd = map(THROTTLE - PitchPIDOutput + RollPIDOutput + YawPIDOutput, 0, 100, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); //BL
  
  Serial.println();
  Serial.print("motA: "); Serial.println(mota); 
  Serial.print("motB: "); Serial.println(motb); 
  Serial.print("motC: "); Serial.println(motc); 
  Serial.print("motD: "); Serial.println(motd);
    
  motA.writeMicroseconds(mota); // CCW 
  motB.writeMicroseconds(motb); // CW 
  motC.writeMicroseconds(motc); // CW
  motD.writeMicroseconds(motd); // CCW
      
  delay(500);
}


int16_t inline ExecutePitchPID(const int16_t& pitch_set_point, const int16_t& measured_pitch) {
  float error;
  static float previous_error = .0;
  static float integral = .0;
  
  float proportional;
  float derivative;
  
  error = static_cast<float>(pitch_set_point - measured_pitch);
  
  proportional = PID_PITCH_P * error;
  derivative = PID_PITCH_D * (error - (previous_error));
  integral += PID_PITCH_I * (previous_error + error)/2;
    
  previous_error = error;
  return static_cast<int16_t>(proportional + integral + derivative);
}

int16_t inline ExecuteRollPID(const int16_t& roll_set_point, const int16_t& measured_roll) {
  float error;
  static float previous_error = .0;
  static float integral = .0;
  
  float proportional;
  float derivative;
  
  error = roll_set_point - measured_roll;
  
  proportional = PID_ROLL_P * error;
  derivative = PID_ROLL_D * (error - (previous_error));
  integral += PID_ROLL_I * (previous_error + error)/2;
  
  previous_error = error;
  
  return proportional + integral + derivative;
}

int16_t inline ExecuteYawPID(const int16_t& yaw_set_point, const int16_t& measured_yaw) {
  float error;
  static float previous_error = .0;
  static float integral = .0;
  
  float proportional;
  float derivative;
  
  error = yaw_set_point - measured_yaw;
  
  proportional = PID_YAW_P * error;
  derivative = PID_YAW_D * (error - (previous_error));
  integral += PID_YAW_I * (previous_error + error)/2;
  
    
  previous_error = error;

  return proportional + integral + derivative;
}

void inline MeasurePitchRollYaw(int16_t& measured_pitch, int16_t& measured_roll, int16_t& measured_yaw) {
  measured_pitch = static_cast<int16_t>(accelgyro.getRotationX());
 // Serial.println( measured_pitch); 
  measured_roll =  static_cast<int16_t>(accelgyro.getRotationY());
  measured_yaw =   static_cast<int16_t>(accelgyro.getRotationZ());
}