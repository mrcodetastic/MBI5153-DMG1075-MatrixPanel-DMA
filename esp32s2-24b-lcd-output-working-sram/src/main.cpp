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

  bus_cfg.pin_wr      = I2S_WR_CLOCK; 
  bus_cfg.invert_pclk = false;

  bus_cfg.pin_d0 = MBI_GCLK;
  bus_cfg.pin_d1 = MBI_DCLK;
  bus_cfg.pin_d2 = MBI_LAT;
  bus_cfg.pin_d3 = ADDR_A_PIN;
  bus_cfg.pin_d4 = ADDR_B_PIN;
  bus_cfg.pin_d5 = ADDR_C_PIN; // blue
  bus_cfg.pin_d6 = ADDR_D_PIN;
  bus_cfg.pin_d7 = ADDR_E_PIN;
  bus_cfg.pin_d8 = MBI_G1; // start of second byte
  bus_cfg.pin_d9 = MBI_B1;
  bus_cfg.pin_d10 = MBI_R1;
  bus_cfg.pin_d11 = MBI_G2;
  bus_cfg.pin_d12 = MBI_B2;
  bus_cfg.pin_d13 = MBI_R2;
  bus_cfg.pin_d14 = MBI_G3;  
  bus_cfg.pin_d15 = MBI_B3; // end of byte 2
  bus_cfg.pin_d16 = MBI_R3; // start of third byte
  bus_cfg.pin_d17 = MBI_G4;
  bus_cfg.pin_d18 = MBI_B4;
  bus_cfg.pin_d19 = MBI_R4;
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

