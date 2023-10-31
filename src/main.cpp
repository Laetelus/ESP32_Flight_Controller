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
unsigned long start_time = micros();
    //f.read_Controller();
    f.level_flight();
    f.motorControls();
    f.mix_motors();
    f.write_motors();
    //f.print_gyro_data(); //Only uncomment when debugging
//! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
// Because of the angle calculation the loop time is getting very important. If the loop time is
// longer or shorter than 4000us the angle calculation is off. If you modify the code make sure
// that the loop time is still 4000us and no longer! 
//! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    // while(micros() - loop_timer < 4000); //250hz                                 
    // loop_timer = micros();
    unsigned long loop_duration = micros() - start_time;
    if (loop_duration > 4000) {
        //Serial.print("Warning: Loop time exceeded 4000us. Loop time: ");
        //Serial.println(loop_duration);
    } else {
        while (micros() - start_time < 4000); // Wait until 4000us have passed since start of loop
    }

}

