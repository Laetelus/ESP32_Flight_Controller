#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>
#include "Flight_Controller.h"

Flight_Controller f; 

// Define the hardware timer
hw_timer_t * timer = NULL;
// Define the semaphore to signal from ISR to loop
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool timeFlag = false; // A flag to indicate when the ISR has been executed

void IRAM_ATTR onTimer() { // This function will run every time the timer is triggered
  portENTER_CRITICAL_ISR(&timerMux);
  timeFlag = true; // Set the flag
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  f.initialize();

  // Initialize hardware timer
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (for 1MHz), count up
  timerAttachInterrupt(timer, &onTimer, true); // Attach the callback function
  timerAlarmWrite(timer, 4000, true); // Set the alarm to trigger every 4000us (4ms = 250Hz)
  timerAlarmEnable(timer); // Enable the timer
}

void loop() {
  if (timeFlag) { // Check if the timer flag has been set
    portENTER_CRITICAL(&timerMux);
    timeFlag = false; // Reset the flag
    portEXIT_CRITICAL(&timerMux);

    f.level_flight();
    f.motorControls();
    f.mix_motors();
    f.write_motors();
    //f.print_gyro_data(); 
  }
}

