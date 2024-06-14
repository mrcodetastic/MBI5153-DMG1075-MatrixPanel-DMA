#include <esp32s2_i2s_lcd_parallel_dma.hpp>
  

#ifndef MATRIX_WIDTH
#define MATRIX_WIDTH  78 
#endif

#ifndef MATRIX_HEIGHT
#define MATRIX_HEIGHT 78 
#endif

// RGB Panel Scan Lines
#define PANEL_SCAN_LINES        20 // panel scan lines / rows
#define PANEL_MBI_LED_CHANS     16  // number of led channels per MBI IC
#define PANEL_MBI_CHAIN_LEN      5  // number of ic's changed for each subpixel

#define PANEL_PHY_RES_X 78
#define PANEL_PHY_RES_Y 78

#define PANEL_MBI_RES_X 80
#define PANEL_MBI_RES_Y 80


/***************************************************************************************/
/*
  // MBI5152 Application Note V1.00- EN

  Section 2: The Setting of Gray Scale 
  The setting of gray scale data describes as below. 
  1. The sequence of input data starts from scan line 1 scan line 2….  scan line M-1 scan line M 
  (M≦16) 
  2. The data sequence of cascaded IC is ICnICn-1…. IC2IC1. 
  3. The data sequence of each channel is ch15ch14…. ch0. 
  4. The data length of each channel is 16-bits, and the default PWM mode is 16-bits. The sequence of gray 
  sacle is bit15bit14bit13…bit0 as figure 4 shows. The 14-bits gray scale can be set through Bit[7]=1 
  in configuration register 1, and the sequence of gray scale data is bit13bit12… bit0 0 0, the 
  last 2-bits (LSB) are set to “0”. 
  5. The frequency of GCLK must be higher than 20% of DCLK to get the correct gray scale data. 
  6. LE executes the data latch to send gray scale data into SRAM. Each 16xN bits data needs a “data latch 
  command”, where N means the number of cascaded driver. 
  7. After the last data latch, it needs at least 50 GCLKs to read the gray scale data into internal display buffer 
  before the Vsync command comes. 
  8. Display is updated immediately when MBI5152 receives the Vsync signal. 
  9. GCLK must keep at low level more than 7ns before MBI5152 receives the Vsync signal. 
  10. The period of dead time (ie. The 1025th GCLK) must be larger than 100ns.

  The gray scale data needs the GCLK to save the data into SRAM. The frequency of GCLK must be higher 
  than 20% of DCLK to get the correct data. 

  // MBI5153
  After the last data latch command, it needs at least 50 GCLKs to read the gray scale data into internal display 
  buffer before the Vsync command comes. And display is updated immediately until MBI5051/52/53 receives 
  the Vsync signal (high pulse of LE pin is sampled by 3-DCLK rising edges), as figure 6 shows.

*/

#define ghost_elimination_ON    0b11   //послесвечение выключенно
#define ghost_elimination_OFF   0   //послесвечение включено
#define gray_scale_13           1   //шкала серого 13 бит
#define gray_scale_14           0   //шкала серого 14 бит

/* GCLK multiplier 
MBI5153 provides a GCLK multiplier function by setting the configuration register1 bit [6]. The default value is set 
to ’0’ for GCLK multiplier disable = 513 required for each row.

GCLK multiplier enabled (configuration register1 bit [6] = 1) 
= 257 gclocks for each row!
*/

#define gclk_multiplier_ON_clks  257   // GCLK Multipler On - You MUST use exactly 257 clocks for each rowscan!
#define gclk_multiplier_OFF_clks 513  // GCLK Multipler Off - You MUST use exactly 513 clocks for each rowscan!


#define gclk_multiplier_ON      1   // GCLK Multipler On - You MUST use exactly 257 clocks for each rowscan!
#define gclk_multiplier_OFF     0   // GCLK Multipler On - You MUST use exactly 513 clocks for each rowscan!
#define current_1               15  //ток на светодиоде
#define current_2               63  //ток на светодиоде
#define current_3               35  //ток на светодиоде
#define current_4               20  //LED current

#define clock_delay             0
#define pwm_hi                  65535  
#define pwm_lo                  0 


/***************************************************************************************/
// For DMA GLCK and Address Line Data
#define BIT_GCLK  (1 << 0)
#define BIT_DCLK  (1 << 1)
#define BIT_LAT   (1 << 2)
#define BIT_A     (1 << 3)
#define BIT_B     (1 << 4)
#define BIT_C     (1 << 5)
#define BIT_D     (1 << 6)
#define BIT_E     (1 << 7)

// For DMA Greyscale and other data
#define BIT_COLOR_POS_START 8
#define BIT_G1   (1 << 8)
#define BIT_B1   (1 << 9)
#define BIT_R1   (1 << 10)
#define BIT_G2   (1 << 11)
#define BIT_B2   (1 << 12)
#define BIT_R2   (1 << 13)
#define BIT_G3   (1 << 14)
#define BIT_B3   (1 << 15)
#define BIT_R3   (1 << 16)
#define BIT_G4   (1 << 17)
#define BIT_B4   (1 << 18)
#define BIT_R4   (1 << 19)
#define BIT_RGB_ALL (BIT_G1 | BIT_B1 | BIT_R1 | BIT_G2 | BIT_B2 | BIT_R2 | BIT_G3 | BIT_B3 | BIT_R3 | BIT_G4 | BIT_B4 | BIT_R4 )

/***************************************************************************************/

#define I2S_WR_CLOCK            GPIO_NUM_9

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




/***************************************************************************************/
DMA_DATA_TYPE *global_buffer; // data of stuff
lldesc_t *dma_ll;
  