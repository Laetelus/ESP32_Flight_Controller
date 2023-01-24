#include <PPMReader.h>
#include <ESP32Servo.h> 
#include <PID_v1.h>
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
#define MID_PULSE_LENGTH 1500 //Neutral pulse length in µs

Servo motA,motB,motC,motD; 
PPMReader ppm(interruptPin, channelAmount);

//Declare variables for PID controllers
double throttleSetpoint, throttleInput, throttleOutput;
double yawSetpoint, yawInput, yawOutput;
double rollSetpoint, rollInput, rollOutput;
double pitchSetpoint, pitchInput, pitchOutput;

PID throttlePID(&throttleInput, &throttleOutput, &throttleSetpoint, 2, 5, 1, DIRECT);
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, 2, 5, 1, DIRECT);
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, 2, 5, 1, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, 2, 5, 1, DIRECT);

int start = 0; //Flag to indicate whether to start

void setup() {
    Serial.begin(115200);

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
    uint_fast16_t sample_time = 20; // in milliseconds
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

    // Scale the input values to a range of 0 to 100
    throttleInput = map(throttle, 1000, 2000, 0, 100);
    yawInput = map(yaw, 1000, 2000, -80, 80);
    rollInput = map(roll, 1000, 2000, -50, 50);
    pitchInput = map(pitch, 1000, 2000, -50, 50);
    
    //Set the setpoints
    throttleSetpoint = 0;
    yawSetpoint = 0;
    rollSetpoint = 0;
    pitchSetpoint = 0;

    //Call the Compute function for each PID controller
    throttlePID.Compute();
    yawPID.Compute();
    rollPID.Compute();
    pitchPID.Compute();

    //Mix the inputs to determine the speed and direction of each motor
    int16_t mota = map(throttleInput + pitchOutput - rollOutput + yawOutput, 0, 100, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); //FR
    int16_t motb = map(throttleInput - pitchOutput - rollOutput - yawOutput, 0, 100, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); //BR
    int16_t motc = map(throttleInput + pitchOutput + rollOutput - yawOutput, 0, 100, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); //FL
    int16_t motd = map(throttleInput - pitchOutput + rollOutput + yawOutput, 0, 100, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); //BL
    
    Serial.println();
    Serial.print("motA: "); Serial.println(mota); Serial.print("motB: "); Serial.println(motb); Serial.print("motC: "); Serial.println(motc); Serial.print("motD: "); Serial.println(motd);


    // Write the motor outputs
    motA.writeMicroseconds(mota);
    motB.writeMicroseconds(motb);
    motC.writeMicroseconds(motc);
    motD.writeMicroseconds(motd);
    delay(500); 

}


