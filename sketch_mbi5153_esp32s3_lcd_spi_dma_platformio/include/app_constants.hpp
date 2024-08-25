/******************************************************************************************
 * @file        app_constants.hpp
 * @author      github.com/mrcodetastic
 * @date        2024
 * @brief       ESP32-S3 implementation for a MBI5135 PWM chip based LED Matrix Panel
 ******************************************************************************************/


#pragma once
#include "hal/gpio_types.h"

// Updated ESP32-S3 Pin GPIO
/* Experimenting with the ESP32-S3 Dev Module has uncovered that ONLY THESE pins seems to work ok
 * use other pins at your risk. Don't use 19, 20, 48 etc.
 * https://api.riot-os.org/group__cpu__esp32__esp32s3.html
 */

#define GPIO_MAPPING_DEFAULT     1
#define GPIO_MAPPING_PCB_DEVS3   2

// Selected mode
//#define GPIO_MAPPING GPIO_MAPPING_DEFAULT
#define GPIO_MAPPING GPIO_MAPPING_PCB_DEVS3


// GPIo48 is the on board RGB LED 
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
  #define MBI_B2                  GPIO_NUM_18  
  #define MBI_R2                  GPIO_NUM_17  

  // Third 1/4 of panel -> 20 rows
  #define MBI_G3                  GPIO_NUM_8  
  #define MBI_B3                  GPIO_NUM_45  
  #define MBI_R3                  GPIO_NUM_46  

  // Forth 1/4 of panel -> 20 rows
  #define MBI_G4                  GPIO_NUM_3    
  #define MBI_B4                  GPIO_NUM_21  
  #define MBI_R4                  GPIO_NUM_47  

#elif (GPIO_MAPPING == GPIO_MAPPING_PCB_DEVS3)  

  #define ADDR_A_PIN              GPIO_NUM_17
  #define ADDR_B_PIN              GPIO_NUM_18
  #define ADDR_C_PIN              GPIO_NUM_5
  #define ADDR_D_PIN              GPIO_NUM_6
  #define ADDR_E_PIN              GPIO_NUM_10

  #define MBI_GCLK                GPIO_NUM_15  // OE PIN IS GCLK apparently
  #define MBI_LAT                 GPIO_NUM_16  //  data/command
  #define MBI_DCLK                GPIO_NUM_7  // data clocking line?
  #define MBI_SRCLK               GPIO_NUM_48   // I assume SR stands for Scan Row??  // When this is HIGH on these boards, output is disabled?

  // First 1/4 of panel -> 20 rows
  #define MBI_G1                  GPIO_NUM_21
  #define MBI_B1                  GPIO_NUM_14 
  #define MBI_R1                  GPIO_NUM_4  

  // Second 1/4 of panel -> 20 rows
  #define MBI_G2                  GPIO_NUM_42   
  #define MBI_B2                  GPIO_NUM_41 
  #define MBI_R2                  GPIO_NUM_47  

  // Third 1/4 of panel -> 20 rows
  #define MBI_G3                  GPIO_NUM_39  
  #define MBI_B3                  GPIO_NUM_38 
  #define MBI_R3                  GPIO_NUM_40 

  // Forth 1/4 of panel -> 20 rows
  #define MBI_G4                  GPIO_NUM_36    
  #define MBI_B4                  GPIO_NUM_35 
  #define MBI_R4                  GPIO_NUM_37


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

// DMA engine (if used)
//#define ESP32_GCLK_DMA_STORAGE_TYPE uint8_t   // DMA output of one uint16_t at a time.
#define ESP32_GREY_DMA_STORAGE_TYPE uint16_t  // DMA output of one uint16_t at a time.

// For DMA GLCK and Address Line Data
#define BIT_GCLK  (1 << 0)
#define BIT_A     (1 << 1)
#define BIT_B     (1 << 2)
#define BIT_C     (1 << 3)
#define BIT_D     (1 << 4)
#define BIT_E     (1 << 5)

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
#define BIT_LAT  (1 << 12)

//#define BIT_DCLK (1 << 13) // generated by PIN WR on LCD device
#define BIT_ALL_RGB    (BIT_G1 | BIT_B1 | BIT_R1 | BIT_G2 | BIT_B2 | BIT_R2 | BIT_G3 | BIT_B3 | BIT_R3 | BIT_G4 | BIT_B4 | BIT_R4)

// RGB Panel Scan Lines
#define PANEL_SCAN_LINES        20 // panel scan lines / rows
#define PANEL_MBI_LED_CHANS     16  // number of led channels per MBI IC
#define PANEL_MBI_CHAIN_LEN      5  // number of ic's changed for each subpixel

#define PANEL_PHY_RES_X 78
#define PANEL_PHY_RES_Y 78

#define PANEL_MBI_RES_X 80
#define PANEL_MBI_RES_Y 80

#define ghost_elimination_ON  0b11 // послесвечение выключенно
#define ghost_elimination_OFF 0    // послесвечение включено
#define gray_scale_13         1    // шкала серого 13 бит
#define gray_scale_14         0    // шкала серого 14 бит

/* 
  GCLK multiplier (grayscale clock multiplier)

  MBI5153 provides a GCLK multiplier function by setting the configuration register1 bit [6]. 
  The default value is set to ’0’ for GCLK multiplier disable   = 513 GCLKS for each row.

  GCLK multiplier enabled (configuration register1 bit [6] = 1) = 257 GCLKS for each row!

*/

#define gclk_multiplier_ON  1  // GCLK Multipler On - You MUST use exactly 257 clocks for each rowscan!
#define gclk_multiplier_OFF 0  // GCLK Multipler On - You MUST use exactly 513 clocks for each rowscan!
#define current_1 15           // ток на светодиоде
#define current_2 63           // ток на светодиоде
#define current_3 35           // ток на светодиоде
#define current_4 20           // LED current

#define pwm_hi 65535
#define pwm_lo 0

#define PHYSICS_SCALE 100
#define OFFSET 0