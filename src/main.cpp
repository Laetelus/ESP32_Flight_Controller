#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>
#include "Flight_Controller.h"
#include "PID_Webserver.h"
#include <WiFi.h>

Flight_Controller fc;
PID_Webserver ws;
delay_time tm; 

void setup() {
  ws.initSPIFFS();
  WiFi.mode(WIFI_STA); // Set WiFi to station mode but don't connect
  fc.initialize(); // Initialize other routines
  ws.Wifi_task(); 
}

void loop() {

  fc.read_Controller();
  fc.readGyroData();
  fc.processIMUData();
  fc.level_flight();
  fc.motorControls();
  fc.mix_motors();
  fc.write_motors();
  //fc.print();

  //4000us (250Hz) cycle
  tm.timer(); 
}
