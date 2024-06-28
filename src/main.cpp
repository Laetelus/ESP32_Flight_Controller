#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>
#include "Flight_Controller.h"
#include "Calibration.h"
#include "PID_Webserver.h"
#include <WiFi.h>

static unsigned long loop_timer;
void setup()
{
  flightController.initialize(); // Initialize other routines
  ws.initSPIFFS();
  WiFi.mode(WIFI_STA); // Set WiFi to station mode but don't connect
  ws.Wifi_task();

  // Load PID values from SPIFFS (if available)
  if (!ws.loadPIDValues())
  {
    Serial.println("No PID values loaded from SPIFFS. Using default values.");
  }

  // Let's start our timer
  loop_timer = micros();
}

void loop()
{
  // static unsigned long loop_timer = micros(); // Initialize loop timer
  unsigned long current_time;

  // Ensure processIMUData is called to update angle_pitch and angle_roll
  flightController.processIMUData(true, true);

  // Execute the main tasks
  flightController.read_Controller();
  flightController.motorControls(); // motorControls calls level_flight and calculate_pid internally
  flightController.mix_motors();
  flightController.write_motors();

  // flightController.print();

  current_time = micros(); // Capture the current time after executing tasks

  // Check if the current loop time exceeds 4000 microseconds
  if (current_time - loop_timer > 4000)
  {
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
  }

  // Ensure the loop runs at 250Hz
  while (micros() - loop_timer < 4000)
    ; // Wait until 10000us have passed (250Hz loop rate)

  loop_timer = micros(); // Reset loop timer for the next iteration
}
