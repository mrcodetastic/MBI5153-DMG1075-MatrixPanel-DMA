#pragma once
#include "hal/gpio_types.h"

// Updated ESP32-S3 Pin GPIO
/* Experimenting with the ESP32-S3 Dev Module has uncovered that ONLY THESE pins seems to work ok
 * use other pins at your risk. Don't use 0, 19, 20, 26-38 etc.
 * https://api.riot-os.org/group__cpu__esp32__esp32s3.html
 */

#define GPIO_MAPPING_DEFAULT     1
#define GPIO_MAPPING_PCB_DEVS3   2
#define GPIO_MAPPING_PCB_DHRUV   3 

// Selected mode
#define GPIO_MAPPING GPIO_MAPPING_DEFAULT


#if (GPIO_MAPPING == GPIO_MAPPING_DEFAULT)

  #define ADDR_A_PIN              GPIO_NUM_5
  #define ADDR_B_PIN              GPIO_NUM_4
  #define ADDR_C_PIN              GPIO_NUM_42
  #define ADDR_D_PIN              GPIO_NUM_7
  #define ADDR_E_PIN              GPIO_NUM_41

  #define MBI_GCLK                GPIO_NUM_1  // OE PIN IS GCLK apparently
  #define MBI_LAT                 GPIO_NUM_6  //  data/command
  #define MBI_DCLK                GPIO_NUM_2  // data clocking line?
  #define MBI_SRCLK               GPIO_NUM_48   // I assume SR stands for Scan Row??  // When this is HIGH on these boards, output is disabled?

  // First 1/4 of panel -> 20 rows
  #define MBI_G1                  GPIO_NUM_40 
  #define MBI_B1                  GPIO_NUM_16 
  #define MBI_R1                  GPIO_NUM_15  

  // Second 1/4 of panel -> 20 rows
  #define MBI_G2                  GPIO_NUM_39    
  #define MBI_B2                  GPIO_NUM_17  
  #define MBI_R2                  GPIO_NUM_18  

  // Third 1/4 of panel -> 20 rows
  #define MBI_G3                  GPIO_NUM_8  
  #define MBI_B3                  GPIO_NUM_45  
  #define MBI_R3                  GPIO_NUM_46  

  // Forth 1/4 of panel -> 20 rows
  #define MBI_G4                  GPIO_NUM_3    
  #define MBI_B4                  GPIO_NUM_21  
  #define MBI_R4                  GPIO_NUM_47  

#elif (GPIO_MAPPING == GPIO_MAPPING_PCB_DEVS3)  
// S3-Dev module header / quick and dirty pcb.

#elif (GPIO_MAPPING == GPIO_MAPPING_PCB_DHRUV)  


#endif


// Spare Unencumbered GPIOS
// https://api.riot-os.org/group__cpu__esp32__esp32s3.html
#define MBI_SPARE               GPIO_NUM_9  
#define MBI_SPARE2              GPIO_NUM_10
#define MBI_SPARE3              GPIO_NUM_11
#define MBI_SPARE4              GPIO_NUM_12
#define MBI_SPARE5              GPIO_NUM_13
#define MBI_SPARE6              GPIO_NUM_14


// -------------------------------------------------------
// Internals specific to code and hardware. Do not change.
// -------------------------------------------------------

// For DMA GCLK and Address Line Data
#define BIT_GCLK (1 << 0)
#define BIT_A (1 << 1)
#define BIT_B (1 << 2)
#define BIT_C (1 << 3)
#define BIT_D (1 << 4)
#define BIT_E (1 << 5)

// For Greyscale data

// For DMA Greyscale and other data
#define BIT_G1   (1 << 0)
#define BIT_B1   (1 << 1)
#define BIT_R1   (1 << 2)
#define BIT_RGB1_CLR (0b111 << 0)
#define BIT_G2   (1 << 3)
#define BIT_B2   (1 << 4)
#define BIT_R2   (1 << 5)
#define BIT_RGB2_CLR (0b111 << 3)
#define BIT_G3   (1 << 6)
#define BIT_B3   (1 << 7)
#define BIT_R3   (1 << 8)
#define BIT_RGB3_CLR (0b111 << 6)
#define BIT_G4   (1 << 9)
#define BIT_B4   (1 << 10)
#define BIT_R4   (1 << 11)
#define BIT_RGB4_CLR (0b111 << 9)




// RGB Panel Scan Lines
#define PANEL_SCAN_LINES        20 // panel scan lines
#define PANEL_MBI_RES_X         80 // How many pixels the MBI chips can support. Sure it's a 78 pixel panel, but 80 pixels are supported
#define PANEL_MBI_LENGTH        5  // number of ic's changed for each subpixel
#define PANEL_PHY_RES_X         78
#define PANEL_PHY_RES_Y         78

