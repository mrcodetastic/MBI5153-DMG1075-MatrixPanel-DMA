Default pin mapping is as follows:

```

  #define ADDR_A_PIN              GPIO_NUM_5
  #define ADDR_B_PIN              GPIO_NUM_4
  #define ADDR_C_PIN              GPIO_NUM_42
  #define ADDR_D_PIN              GPIO_NUM_7
  #define ADDR_E_PIN              GPIO_NUM_41

  #define MBI_GCLK                GPIO_NUM_1  
  #define MBI_LAT                 GPIO_NUM_6  
  #define MBI_DCLK                GPIO_NUM_2  
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
```
