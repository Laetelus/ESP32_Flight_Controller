#include <PPMReader.h>
#include <ESP32Servo.h> 
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050.h"


// Initialize a PPMReader on digital pin 3 with 8 expected channels.
byte interruptPin = 13; // PPM pin connector 
byte channelAmount = 4; // Number of channels to use 

MPU6050 accelgyro; 

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_READABLE_ACCELGYRO
// Define motor pins 
#define MOTOR_1_PIN 14
#define MOTOR_2_PIN 27
#define MOTOR_3_PIN 26
#define MOTOR_4_PIN 25

#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
#define MID_PULSE_LENGTH 1500 //Neutral pulse length in µs

Servo motA,motB,motC,motD; 
PPMReader ppm(interruptPin, channelAmount);

//Declare variables for PID controllers
double throttleSetpoint = 0, throttleInput, throttleOutput;
double yawSetpoint = 0, yawInput, yawOutput;
double rollSetpoint = 0, rollInput, rollOutput;
double pitchSetpoint = 0, pitchInput, pitchOutput;

//Do not exceed kp > 2, ki, > 0.5, kd > 2  
double kp_pitch = 0 , ki_pitch = 0, kd_pitch= 0; 
double kp_roll = 0,  ki_roll= 0, kd_roll = 0; 
double kp_yaw = 0, ki_yaw = 0, kd_yaw = 0;   



PID throttlePID(&throttleInput, &throttleOutput, &throttleSetpoint, kp_pitch, ki_pitch, kd_pitch, DIRECT);
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, kp_yaw, ki_yaw, kd_yaw, DIRECT);
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, kp_roll, ki_roll, kd_roll, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, kp_pitch, ki_pitch, kd_pitch, DIRECT);

void IMU_DATA();

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
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


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

    //Initialize the PID controllers
    throttlePID.SetMode(AUTOMATIC);
    yawPID.SetMode(AUTOMATIC);
    rollPID.SetMode(AUTOMATIC);
    pitchPID.SetMode(AUTOMATIC);
        // Set the sample time
    int16_t sample_time = 50; // in milliseconds
    throttlePID.SetSampleTime(sample_time);
    rollPID.SetSampleTime(sample_time);
    pitchPID.SetSampleTime(sample_time);
    yawPID.SetSampleTime(sample_time);

    //set the output limits
    // throttlePID.SetOutputLimits(MIN_PULSE_LENGTH,MAX_PULSE_LENGTH);
    // rollPID.SetOutputLimits(MIN_PULSE_LENGTH,MAX_PULSE_LENGTH); 
    // pitchPID.SetOutputLimits(MIN_PULSE_LENGTH,MAX_PULSE_LENGTH); 
    // yawPID.SetOutputLimits(MIN_PULSE_LENGTH,MAX_PULSE_LENGTH); 
}

void loop() {
    //Read the raw channel values
    int16_t throttle = ppm.rawChannelValue(3);
    int16_t yaw = ppm.rawChannelValue(4);
    int16_t roll = ppm.rawChannelValue(1);
    int16_t pitch = ppm.rawChannelValue(2);
    
    pitch = static_cast<int16_t>(accelgyro.getRotationX());
    roll  = static_cast<int16_t>(accelgyro.getRotationY());
    yaw  = static_cast<int16_t>(accelgyro.getRotationZ());

    // Scale the input values to a range of 0 to 100
    throttleInput = map(throttle, 1000, 2000, 0, 100);
    yawInput = map(yaw, 1000, 2000, -80, 80);
    Serial.println(yawInput); //yaw input depends on imu... test 
    rollInput = map(roll, 1000, 2000, -50, 50);
    pitchInput = map(pitch, 1000, 2000, -50, 50);
 
    //Mix the inputs to determine the speed and direction of each motor
    int16_t mota = map(throttleInput + pitchOutput - rollOutput + yawOutput, 0, 100, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); //FR
    int16_t motb = map(throttleInput - pitchOutput - rollOutput - yawOutput, 0, 100, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); //BR
    int16_t motc = map(throttleInput + pitchOutput + rollOutput - yawOutput, 0, 100, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); //FL
    int16_t motd = map(throttleInput - pitchOutput + rollOutput + yawOutput, 0, 100, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); //BL

    //Call the Compute function for each PID controller
    throttlePID.Compute();
    yawPID.Compute();
    rollPID.Compute();
    pitchPID.Compute();
    IMU_DATA(); 
    
    Serial.println();
    Serial.print("motA: "); Serial.println(mota); Serial.print("motB: "); Serial.println(motb); Serial.print("motC: "); Serial.println(motc); Serial.print("motD: "); Serial.println(motd);

    // Write the motor outputs
    motA.writeMicroseconds(mota);
    motB.writeMicroseconds(motb);
    motC.writeMicroseconds(motc);
    motD.writeMicroseconds(motd);
    delay(500); 

}



void IMU_DATA(){ 
#ifdef DEBUG
        // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    accelgyro.getAcceleration(&ax, &ay, &az);
    accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif
#endif
}
