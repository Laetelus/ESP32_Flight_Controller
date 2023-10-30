#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>
#include "Flight_Controller.h"

int loop_timer; 
Flight_Controller f; 

void setup() {
    f.initialize();
}

void loop() {
    
    f.read_Controller();
    f.level_flight();
    f.motorControls();
    f.mix_motors();
    f.write_motors();
    //f.print_gyro_data();
    while(micros() - loop_timer < 4000);                                 
    loop_timer = micros();
}
