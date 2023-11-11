#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>
#include "Flight_Controller.h"

Flight_Controller f; 

// Task handle for the readController task
TaskHandle_t Task1;

// Task function that reads controller data
void readControllerTask(void * parameter){
    for(;;){
        f.read_Controller(); // Read controller data
        vTaskDelay(pdMS_TO_TICKS(4)); // 4ms delay for 250Hz loop rate
    }
}

void setup() {
    f.initialize();

    // Create a task that will be executed in the readControllerTask function, on core 0
    xTaskCreatePinnedToCore(
        readControllerTask, /* Task function. */
        "readControllerTask", /* Name of the task. */
        10000, /* Stack size of task */
        NULL, /* parameter of the task */
        1, /* priority of the task */
        &Task1, /* Task handle to keep track of created task */
        0); /* pin task to core 0 */
}

void loop() {
    static int loop_timer = micros();

    f.level_flight();
    f.motorControls();
    f.mix_motors();
    f.write_motors();
    //f.print_gyro_data();
    
    // Ensure loop runs at 4000us (250Hz) cycle
    while(micros() - loop_timer < 4000);
    loop_timer = micros();
}
