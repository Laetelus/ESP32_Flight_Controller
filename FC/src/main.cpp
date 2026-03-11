#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>
#include "Flight_Controller.h"
#include "Calibration.h"
#include "PID_Webserver.h"
#include <WiFi.h>
#include "IMU.h"
#include "Motors.h"
#include "PID.h"
#include "PID_Webserver.h"  

IMU imu;
Motors mot; 
PID pid;
FC fc(imu, pid, mot);
Calibration cal(imu);
PID_Webserver ws(fc, pid);

static unsigned long loop_timer;

void setup()
{
  Serial.begin(115200);
  pinMode(2, OUTPUT);

  fc.initialize();
  ws.initSPIFFS();

  if (!ws.loadPIDValues())
  {
    Serial.println("No PID values loaded from SPIFFS. Using default values.");
  }

  WiFi.mode(WIFI_STA);
  ws.Wifi_task();

  loop_timer = micros();
}

void loop()
{
  // static unsigned long loop_timer = micros(); // Initialize loop timer
  unsigned long current_time;

  fc.run(); 
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
