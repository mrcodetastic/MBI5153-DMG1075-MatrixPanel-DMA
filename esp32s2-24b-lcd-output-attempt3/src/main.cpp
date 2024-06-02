#include <Arduino.h>
#include <main.hpp>

// put function declarations here:
int myFunction(int, int);

config_t bus_cfg;

void setup() {

  bus_cfg.pin_wr      = I2S_WR_CLOCK; // GPIO 9
  bus_cfg.invert_pclk = false;

  // Do not change this order at all.
  /*
  bus_cfg.pin_d0 = MBI_GCLK;
  bus_cfg.pin_d1 = MBI_DCLK;
  bus_cfg.pin_d2 = MBI_LAT;
  bus_cfg.pin_d3 = ADDR_A_PIN;
  bus_cfg.pin_d4 = ADDR_B_PIN;
  bus_cfg.pin_d5 = ADDR_C_PIN;
  bus_cfg.pin_d6 = ADDR_D_PIN;
  bus_cfg.pin_d7 = ADDR_E_PIN;
  bus_cfg.pin_d8 = MBI_G1;
  bus_cfg.pin_d9 = MBI_B1;
  bus_cfg.pin_d10 = MBI_R1;
  bus_cfg.pin_d11 = MBI_G2;
  bus_cfg.pin_d12 = MBI_B2;
  bus_cfg.pin_d13 = MBI_R2;
  bus_cfg.pin_d14 = MBI_G3;
  bus_cfg.pin_d15 = MBI_B3;
  bus_cfg.pin_d16 = MBI_R3;
  bus_cfg.pin_d17 = MBI_G4;
  bus_cfg.pin_d18 = MBI_B4;
  bus_cfg.pin_d19 = MBI_R4;
  bus_cfg.pin_d20 = -1;      
  bus_cfg.pin_d21 = -1;        
  bus_cfg.pin_d22 = -1;          
  bus_cfg.pin_d23 = -1;     
  */

  bus_cfg.pin_d0 = GPIO_NUM_39;
  bus_cfg.pin_d1 = -1;
  bus_cfg.pin_d2 = -1;
  bus_cfg.pin_d3 = -1;
  bus_cfg.pin_d4 = -1;
  bus_cfg.pin_d5 = -1;
  bus_cfg.pin_d6 = -1;
  bus_cfg.pin_d7 = -1;
  bus_cfg.pin_d8 = GPIO_NUM_40; // second byte
  bus_cfg.pin_d9 = -1;
  bus_cfg.pin_d10 = -1;
  bus_cfg.pin_d11 = -1;
  bus_cfg.pin_d12 = -1;
  bus_cfg.pin_d13 = -1;
  bus_cfg.pin_d14 = -1;
  bus_cfg.pin_d15 = -1;
  bus_cfg.pin_d16 = GPIO_NUM_37; // third byte
  bus_cfg.pin_d17 = -1;
  bus_cfg.pin_d18 = -1;
  bus_cfg.pin_d19 = -1;
  bus_cfg.pin_d20 = -1;      
  bus_cfg.pin_d21 = -1;        
  bus_cfg.pin_d22 = -1;          
  bus_cfg.pin_d23 = -1;  


  i2s_lcd_setup(bus_cfg);

  dma_allocate();

  dma_start();





}

void loop() {
  // put your main code here, to run repeatedly:
}

