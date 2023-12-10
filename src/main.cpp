#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>
#include "Flight_Controller.h"
#include <esp_task_wdt.h>

Flight_Controller f; 

// Task handle for the readController task
//TaskHandle_t Task1;

// // Task function that reads controller data
// void readControllerTask(void *parameter){
//     for(;;){
//         esp_task_wdt_reset();
//         f.read_Controller(); // Read controller data
//         vTaskDelay(pdMS_TO_TICKS(4)); // 4ms delay for 250Hz loop rate
//     }
// }

void setup() {
    f.initialize();
    // esp_task_wdt_init(3, true); 
    // // Create a task that will be executed in the readControllerTask function, on core 0
    // xTaskCreatePinnedToCore(
    //     readControllerTask, /* Task function. */
    //     "readControllerTask", /* Name of the task. */
    //     10000, /* Stack size of task */
    //     NULL, /* parameter of the task */
    //     2, /* priority of the task, higher # increases priority */
    //     &Task1, /* Task handle to keep track of created task */
    //     0); /* pin task to core 0 */
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

    // If the time taken is more than 4000 microseconds, print a warning message
    if (time_taken > 4000) {
        //Serial.print("Warning: Loop exceeded 250Hz limit. Time taken (us): ");
        //Serial.println(time_taken);
        digitalWrite(2,HIGH); 
        delay(100); 
        digitalWrite(2,LOW);
        delay(100);
    }

    // Ensure loop runs at 4000us (250Hz) cycle
    while(micros() - loop_timer < 4000);
    loop_timer = micros();
}

// void loop() {
//     static int loop_timer = micros();
//     f.read_Controller();
//     f.level_flight();
//     f.motorControls();
//     f.mix_motors();
//     f.write_motors();
//     f.parse_data();
    
//     // Ensure loop runs at 4000us (250Hz) cycle
//     while(micros() - loop_timer < 4000);
//     loop_timer = micros();
// }
