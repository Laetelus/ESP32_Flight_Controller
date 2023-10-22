#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>
#include "Flight_Controller.h"

Flight_Controller f; 

void setup() {
    f.initialize();
}

void loop() {
    f.read_Controller();
    f.level_flight();
    f.motorControls();
    f.mix_motors();
    delayMicroseconds(4000); 
    f.write_motors();
    //f.print_gyro_data();
}
