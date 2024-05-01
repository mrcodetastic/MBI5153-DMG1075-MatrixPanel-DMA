// WORK IN PROGRESS
// Step 1. Malloc RAM using DMA - easy
// Step 2. Create a pile of crap to send. - easier
// Step 3. Send it using 20bit parallel - 
// Step 4. Test test and test

#include <Arduino.h>
#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "sdkconfig.h"
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>

#include "esp32_i2s_parallel_dma.hpp"

DMA_ATTR uint32_t *dma_gpio_data;

const int frame_size = sizeof(uint32_t)*80*20*16;

// put function declarations here:
int myFunction(int, int);


void setup() {

    Serial.begin(115200);
    //https://github.com/espressif/arduino-esp32/issues/8080
    //https://community.platformio.org/t/esp32-c3-framework-arduino-serial-print-usb/30464
    Serial.setDebugOutput(true); // don't sent debug to UART pins

    esp_log_level_set("*", ESP_LOG_VERBOSE); // Set log level to include ESP_LOGD messages

    // Need to delay because ESP32-S2 debug via USB Serial takes ages to connect before messages get sent.
    delay(8000);
    Serial.println("Starting....");
    esp_task_wdt_deinit();


    // put your setup code here, to run once:
    dma_gpio_data = (uint32_t *)heap_caps_aligned_alloc(64, frame_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_32BIT);
    assert(dma_gpio_data != nullptr);

    delay(1000);
    ESP_LOGI("ESP32-S2-DMA", "Allocated %d bytes DMA memory from PSRAM.", frame_size);    

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Testing 123");
  ESP_LOGD("ESP32-S2-DMA", "Debug Test!");
  delay(1000);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}