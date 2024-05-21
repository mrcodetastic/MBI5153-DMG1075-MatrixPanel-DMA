#pragma once

  #define MBI_GCLK                GPIO_NUM_39  // OE PIN IS GCLK apparently
  #define MBI_DCLK                GPIO_NUM_40  // data clocking line?
  #define MBI_LAT                 GPIO_NUM_37  //  data/command  
  

  #define ADDR_A_PIN              GPIO_NUM_38
  #define ADDR_B_PIN              GPIO_NUM_35
  #define ADDR_C_PIN              GPIO_NUM_36
  #define ADDR_D_PIN              GPIO_NUM_33
  #define ADDR_E_PIN              GPIO_NUM_34

  
  // First 1/4 of panel -> 20 rows
  #define MBI_G1                  GPIO_NUM_18 
  #define MBI_B1                  GPIO_NUM_21 
  #define MBI_R1                  GPIO_NUM_16  

  // Second 1/4 of panel -> 20 rows
  #define MBI_G2                  GPIO_NUM_17    
  #define MBI_B2                  GPIO_NUM_1  
  #define MBI_R2                  GPIO_NUM_2  

  // Third 1/4 of panel -> 20 rows
  #define MBI_G3                  GPIO_NUM_3  
  #define MBI_B3                  GPIO_NUM_4  
  #define MBI_R3                  GPIO_NUM_5  

  // Forth 1/4 of panel -> 20 rows
  #define MBI_G4                  GPIO_NUM_6    
  #define MBI_B4                  GPIO_NUM_7  
  #define MBI_R4                  GPIO_NUM_8

  #define I2S_WR_CLOCK            GPIO_NUM_9