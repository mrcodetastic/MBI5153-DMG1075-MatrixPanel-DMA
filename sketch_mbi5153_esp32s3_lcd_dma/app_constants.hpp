#pragma once
#include "hal/gpio_types.h"

// DMA engine 
#define ESP32_I2S_DMA_STORAGE_TYPE uint8_t // DMA output of one uint16_t at a time.

#define BIT_GCLK (1 << 0)
#define BIT_A (1 << 1)
#define BIT_B (1 << 2)
#define BIT_C (1 << 3)
#define BIT_D (1 << 4)
#define BIT_E (1 << 5)
#define BIT_LAT (1 << 6)
#define BIT_DCLK (1 << 7)

// RGB Panel Scan Lines
#define PANEL_SCAN_LINES        20 // panel scan lines
#define PANEL_MBI_LENGTH        5  // number of ic's changed for each subpixel

// Updated ESP32-S3 Pin GPIO
/* Experimenting with the ESP32-S3 Dev Module has uncovered that ONLY THESE pins seems to work ok
 * use other pins at your risk. Don't use 19, 20, 48 etc.
 */
#define ADDR_A_PIN              GPIO_NUM_5
#define ADDR_B_PIN              GPIO_NUM_4
#define ADDR_C_PIN              GPIO_NUM_42
#define ADDR_D_PIN              GPIO_NUM_7
#define ADDR_E_PIN              GPIO_NUM_41

#define MBI_GCLK                GPIO_NUM_1  // OE PIN IS GCLK apparently
#define MBI_LAT                 GPIO_NUM_6  //  data/command
#define MBI_DCLK                GPIO_NUM_2  // data clocking line?
#define MBI_SDI                 GPIO_NUM_40   // data G1 Pin only for now for now

// First 1/4 of panel -> 20 rows
#define MBI_G1                  MBI_SDI 
#define MBI_B1                  GPIO_NUM_16 
#define MBI_R1                  GPIO_NUM_15  

// Second 1/4 of panel -> 20 rows
#define MBI_G2                  GPIO_NUM_38    
#define MBI_B2                  GPIO_NUM_17  
#define MBI_R2                  GPIO_NUM_18  

// Third 1/4 of panel -> 20 rows
#define MBI_G3                  GPIO_NUM_8  
#define MBI_B3                  GPIO_NUM_45  
#define MBI_R3                  GPIO_NUM_37  

// Forth 1/4 of panel -> 20 rows
#define MBI_G4                  GPIO_NUM_3    
#define MBI_B4                  GPIO_NUM_21  
#define MBI_R4                  GPIO_NUM_47  

#define MBI_SRCLK               GPIO_NUM_48   // Not used. I assume SR stands for Scan Row??  // When this is HIGH on these boards, Address Lines are disabled?
#define MBI_SPARE               GPIO_NUM_12   // Not used.

