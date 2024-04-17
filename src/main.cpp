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

static unsigned long loop_timer = esp_timer_get_time();

void WiFiTask(void *parameter) {
  for(;;) { // Infinite loop
    if (fc.areMotorsOff()) {
      ws.initWiFi();
      ws.checkWiFiConnection();
    } else {
      ws.disconnect_wifi();
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay to prevent the task from using all CPU time
  }
}

void setup() {
  ws.initSPIFFS();
  WiFi.mode(WIFI_STA); // Set WiFi to station mode but don't connect
  fc.initialize(); // Initialize other routines

  // Create a task for WiFi management
  xTaskCreatePinnedToCore(
    WiFiTask, /* Task function */
    "WiFiTask", /* Name of task */
    10000, /* Stack size of task */
    NULL, /* Parameter of the task */
    1, /* Priority of the task */
    NULL, /* Task handle to keep track of created task */
    0); /* Core where the task should run */
}

void loop() {

  fc.read_Controller();
  fc.readGyroData();
  fc.processIMUData();
  fc.level_flight();
  fc.motorControls();
  fc.mix_motors();
  fc.write_motors();
  fc.print();

  // Check the total time taken for this loop
  unsigned long time_taken = esp_timer_get_time() - loop_timer;

  // If the time taken is more than 4000 microseconds, blink led.
  if (time_taken > 4000) {
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
  }

  // Ensure loop runs at 4000us (250Hz) cycle
  while (esp_timer_get_time() - loop_timer < 4000);
  loop_timer = esp_timer_get_time();
}
