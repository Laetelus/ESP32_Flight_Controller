#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>
#include "Flight_Controller.h"
#include <WiFi.h>

Flight_Controller f;

void setup()
{
  f.initSPIFFS();
  WiFi.mode(WIFI_STA); // initialize WiFi mode but don't connect yet
  f.initialize(); //initialize other routines
}

void loop()
{
  // Manage WiFi based on the motors' state
  if (f.areMotorsOff())
  {
    f.initWiFi();
    f.checkWiFiConnection();
  }
  else
  {
    f.disconnect_wifi();
  }

  static unsigned long loop_timer = micros();

  f.read_Controller();
  f.readGyroData();
  f.processIMUData();
  f.level_flight();
  f.motorControls();
  f.mix_motors();
  f.write_motors();

  //f.print();

  // Check the total time taken for this loop
  unsigned long time_taken = micros() - loop_timer;

  // If the time taken is more than 4000 microseconds, blink led.
  if (time_taken > 4000)
  {
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
  }

  // Ensure loop runs at 4000us (250Hz) cycle
  while (micros() - loop_timer < 4000)
    ;
  loop_timer = micros();
}
