#include <Wire.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "Kalman.h"

// MPU6050 variables
int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;

// Kalman filter instances for roll and pitch
Kalman kalmanRoll;
Kalman kalmanPitch;

// Timing variables
unsigned long prevTime;

void setup() {
    Serial.begin(115200);
    initializeI2CBus();
    pinMode(2, OUTPUT); // LED for status indication
    Serial.println("MPU6050 initialization complete.");

    // Initialize timing
    prevTime = micros();
}

void loop() {
    // Read and process IMU data
    processIMUData();

    // Calculate time step
    unsigned long currentTime = micros();
    float dt = (currentTime - prevTime) / 1000000.0f; // Convert micros to seconds
    prevTime = currentTime;

    // Get angles using the Kalman filter
    float roll = kalmanRoll.getAngle(raw_ax / 16384.0 * 180/PI, raw_gx / 131.0, dt);
    float pitch = kalmanPitch.getAngle(raw_ay / 16384.0 * 180/PI, raw_gy / 131.0, dt);

    // Print filtered angles for monitoring
    Serial.print(roll);
    Serial.print(",");
    Serial.println(pitch);

    delay(10); // Adjust based on your timing requirements
}

void processIMUData() {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B); // Start reading at the accelerometer output register
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14); // Request 14 bytes from MPU6050

    // Read accelerometer and gyroscope data
    raw_ax = Wire.read() << 8 | Wire.read();
    raw_ay = Wire.read() << 8 | Wire.read();
    raw_az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // Skip temperature data
    raw_gx = Wire.read() << 8 | Wire.read();
    raw_gy = Wire.read() << 8 | Wire.read();
    raw_gz = Wire.read() << 8 | Wire.read();
}

void initializeI2CBus() {
    Wire.begin();
    Wire.setClock(400000); // Set I2C frequency to 400kHz
    Wire.beginTransmission(0x68);
    Wire.write(0x6B); // Wake up MPU6050
    Wire.write(0x00);
    Wire.endTransmission(true);
}
