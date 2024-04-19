#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>
#include "Flight_Controller.h"
#include "Calibration.h"
#include "PID_Webserver.h"
#include <WiFi.h>

void setup()
{
  ws.initSPIFFS();
  WiFi.mode(WIFI_STA);           // Set WiFi to station mode but don't connect
  flightController.initialize(); // Initialize other routines
  ws.Wifi_task();
}

void loop()
{

  flightController.read_Controller();
  flightController.readGyroData();
  flightController.processIMUData();
  flightController.level_flight();
  flightController.motorControls();
  flightController.mix_motors();
  flightController.write_motors();
  // flightController.print();

  // 4000us (250Hz) cycle
  flightController.timer();
}
