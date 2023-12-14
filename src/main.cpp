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
    static unsigned long loop_timer = micros();

    f.read_Controller();
    f.level_flight();
    f.motorControls();
    f.mix_motors();
    f.write_motors();
    
    //Testing purposes. 
    //f.parse_data();

    // Check the total time taken for this loop
    unsigned long time_taken = micros() - loop_timer;

    // If the time taken is more than 4000 microseconds, blink led.
    if (time_taken > 4000) {
        digitalWrite(2,HIGH); 
        delay(50); 
        digitalWrite(2,LOW);
        delay(50);
    }

    // Ensure loop runs at 4000us (250Hz) cycle
    while(micros() - loop_timer < 4000);
    loop_timer = micros();
}

