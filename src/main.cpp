
#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>
#include "Flight_Controller.h" 
Flight_Controller f; 

void setup() {
    // Initialize the quadcopter
    f.initialize();
}

void loop() {
    // Your main loop code here
    f.read_Controller();
    f.level_flight();
    f.motorControls();
    f.calculate_pid();
    f.outputMotors();
    f.print_gyro_data();
}
