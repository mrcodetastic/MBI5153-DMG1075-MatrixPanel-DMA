#include <Arduino.h>
#include <main.hpp>
#include <esp_log.h>

// put function declarations here:
int myFunction(int, int);

config_t bus_cfg;

void setup() {

    Serial.begin(115200);
    //https://github.com/espressif/arduino-esp32/issues/8080
    //https://community.platformio.org/t/esp32-c3-framework-arduino-serial-print-usb/30464
    Serial.setDebugOutput(true); // don't sent debug to UART pins
    
    for (int delaysec = 7; delaysec > 0; delaysec--)
    {
      delay(1000);
      ESP_LOGI("I2S-DMA", "Starting in %d...", delaysec);
    }

  bus_cfg.bus_freq = 2*1000*1000;
  bus_cfg.parallel_width = 24;

  bus_cfg.pin_wr      = GPIO_NUM_39; 
  bus_cfg.invert_pclk = false;

  bus_cfg.pin_d0 = GPIO_NUM_40;
  bus_cfg.pin_d1 = GPIO_NUM_37;
  bus_cfg.pin_d2 = -1;
  bus_cfg.pin_d3 = -1;
  bus_cfg.pin_d4 = -1;
  bus_cfg.pin_d5 = -1; // blue
  bus_cfg.pin_d6 = -1;
  bus_cfg.pin_d7 = -1;
  bus_cfg.pin_d8 = GPIO_NUM_38; // start of second byte
  bus_cfg.pin_d9 = -1;
  bus_cfg.pin_d10 = -1;
  bus_cfg.pin_d11 = -1;
  bus_cfg.pin_d12 = -1;
  bus_cfg.pin_d13 = -1;
  bus_cfg.pin_d14 = GPIO_NUM_35;  
  bus_cfg.pin_d15 = GPIO_NUM_36; // end of byte 2
  bus_cfg.pin_d16 = GPIO_NUM_33; // start of third byte
  bus_cfg.pin_d17 = -1;
  bus_cfg.pin_d18 = -1;
  bus_cfg.pin_d19 = -1;
  bus_cfg.pin_d20 = -1;      
  bus_cfg.pin_d21 = -1;        
  bus_cfg.pin_d22 = -1;          
  bus_cfg.pin_d23 = GPIO_NUM_34;  // end of third byte



  i2s_lcd_setup_v2(bus_cfg);

  dma_allocate_v2(bus_cfg);

  dma_start_v2();


}

void loop() {
  // put your main code here, to run repeatedly:
     ESP_LOGI("main", "We're still alive...");
     delay(2000);
}

