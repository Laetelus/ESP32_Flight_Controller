//TODO: Add start_stop_takeoff for motors 
//TODO: Test mixer algorithm for PITCH,YAW,ROLL,thrust
//TODO: Create a PID algorithm for PITCH,YAW,ROLL,thrust
//TODO: Create failsafe algorithm using MPU6050 


#include <PPMReader.h>
#include<ESP32Servo.h> 
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
#define MID_PULSE_LENGTH 1400 //Neutral pulse length in µs

Servo motA,motB,motC,motD; 
PPMReader ppm(interruptPin, channelAmount);

 unsigned THROTTLE = 0; 
 unsigned YAW = 0; 
 unsigned ROLL =  0; 
 unsigned PITCH = 0; 

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

}

void loop() {
 
  THROTTLE = ppm.rawChannelValue(2); // Left Joystick up and down 
  YAW = ppm.rawChannelValue(4); // Left joystick Left and right   
  ROLL = ppm.rawChannelValue(1); // Right joystick Left and right 
  PITCH =  ppm.rawChannelValue(3); // Right joystick Up and down 

  Serial.println();
  Serial.print("THROTTLE: " + String(THROTTLE) + "\n");
  Serial.print("YAW: " + String(YAW) + "\n");
  Serial.print("PITCH: " + String(PITCH) + "\n");
  Serial.print("ROLL: " + String(ROLL) + "\n");

      // Scale the input values to a range of 0 to 100
    THROTTLE = map(THROTTLE, 1000, 2000, 0, 100);
    YAW = map(YAW, 1000, 2000, -50, 50);
    PITCH = map(PITCH, 1000, 2000, -50, 50);
    ROLL = map(ROLL, 1000, 2000, -50, 50);

    // Mix the input values to determine the speed and direction of each motor
    int mota = THROTTLE + YAW + PITCH - ROLL;
    int motb = THROTTLE - YAW + PITCH + ROLL;
    int motc = THROTTLE - YAW - PITCH - ROLL;
    int motd = THROTTLE + YAW - PITCH + ROLL;

    // Scale the output values to the range of MIN_PULSE_LENGTH to MAX_PULSE_LENGTH
    mota = map(mota, 0, 100, MAX_PULSE_LENGTH, MIN_PULSE_LENGTH); 
    motb = map(motb, 0, 100, MAX_PULSE_LENGTH, MIN_PULSE_LENGTH); 
    motc = map(motc, 0, 100, MAX_PULSE_LENGTH, MIN_PULSE_LENGTH);
    motd = map(motd, 0, 100, MAX_PULSE_LENGTH, MIN_PULSE_LENGTH);

    // Updated which propellers are CCW or CW
    motA.writeMicroseconds(motb); //CCW 
    motB.writeMicroseconds(motd); //CW
    motC.writeMicroseconds(mota); //CW
    motD.writeMicroseconds(motc); //CCW
    Serial.println();
    Serial.print("motA: "); Serial.println(mota); Serial.print("motB: "); Serial.println(motb); Serial.print("motC: "); Serial.println(motc); Serial.print("motD: "); Serial.println(motd);
   
    delay(500);
}
