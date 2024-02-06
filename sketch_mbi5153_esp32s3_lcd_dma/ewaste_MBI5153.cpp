#include "ewaste_MBI5153.hpp"
#include "app_constants.hpp"
#include "Arduino.h"
#include <iostream>

static const char *TAG = "mbi5153";

// From the main .ino file
extern uint16_t *greyscale_gpio_data; 
extern size_t    greyscale_buffer_size;


/* clocking the MBI5153 driver data line*/
void mbi_clock(int clock) {
  while (clock--) {
    gpio_set_level((gpio_num_t)MBI_DCLK, 1);
    gpio_set_level((gpio_num_t)MBI_GCLK, 1); // gclk is overridden by DMA, so this won't do anything anyway. 
    esp_rom_delay_us(clock_delay);
    gpio_set_level((gpio_num_t)MBI_DCLK, 0);
    gpio_set_level((gpio_num_t)MBI_GCLK, 0);
  }
}

/*pre-activation command - sent before sending configuration register data*/
void mbi_pre_active() {
  ESP_LOGD(TAG, "Send MBI Pre-Active.");

  gpio_set_level((gpio_num_t)MBI_LAT, 1);
  mbi_clock(14);
  gpio_set_level((gpio_num_t)MBI_LAT, 0);

  /* LE/LAT should be low for any rising edge of DCLK */
  mbi_clock(2);
}

/*vertical sync - updates frame data on outputs, used in conjunction with vertical scan*/
// 9. GCLK must keep at low level more than 7ns before MBI5152 receives the Vsync signa
void mbi_v_sync() {
  ESP_LOGD(TAG, "Send MBI Vert Sync.");
  gpio_set_level((gpio_num_t)MBI_LAT, 1);
  mbi_clock(2);
  gpio_set_level((gpio_num_t)MBI_LAT, 0);
}

/*soft reset*/
void mbi_soft_reset() {
  
// “Software reset” command makes MBI5153 go back to the initial state except configuration register value. After this 
// command is received, the output channels will be turned off and will display again with last gray-scale value after 
// new “Vsync” command is received.
  ESP_LOGD(TAG, "Send MBI Soft Reset.");

  gpio_set_level((gpio_num_t)MBI_LAT, 1);
  mbi_clock(10);
  gpio_set_level((gpio_num_t)MBI_LAT, 0);
}


/*настройка регистра конфигурации*/
//void mbi_configuration(uint8_t ghost_elimination, uint8_t line_num, uint8_t gray_scale, uint8_t gclk_multiplier, uint8_t current) {
void mbi_configuration_1() {  
  uint16_t config_reg1_val = 0;
  //config_reg1_val = (config_reg1_val | (ghost_elimination << 15) | (line_num << 8) | (gray_scale << 7) | (gclk_multiplier << 6) | (current));

  //Documentation says set bits E and F of Config1 Reg to 1
  config_reg1_val = (config_reg1_val | (ghost_elimination_ON << 14) | ((PANEL_SCAN_LINES-1) << 8) | (gray_scale_14 << 7) | (gclk_multiplier_OFF << 6) | (current_1));

  ESP_LOGD(TAG, "Reg value. %u", config_reg1_val);

  for (int i = 0; i < PANEL_MBI_LENGTH; i++)
    mbi_send_config(config_reg1_val, (i == (PANEL_MBI_LENGTH - 1)));  // on last panel, latch
}

/*настройка регистра конфигурации*/
void mbi_configuration_2() 
{
  // F E D C B A 9 8 7 6 5 4 3 2 1 0
  // uint16_t config_reg2_val = 0b0001000000010000; // default value apparently

  uint16_t config_reg2_val = 0b1001000000011110; // removes ghosting except for red
  
  ESP_LOGD(TAG, "Reg2 value. %u", config_reg2_val);
  
  for (int i = 0; i < PANEL_MBI_LENGTH; i++)
    mbi_send_config(config_reg2_val, (i == (PANEL_MBI_LENGTH - 1)), true);  // on last panel, latch
}


/*отправка регистра конфигурации*/
/* Need to set config for all MBI input on the panel at once as they all share the same latch pin */
void mbi_send_config(uint16_t config_reg, bool latch, bool reg2) {
  //  ESP_LOGD(TAG, "Send MBI config");

  int latch_trigger_point = (reg2) ? 9:5; 

  bool config_bit[16] = { 0 };
  int data_mask = 0x1;
  for (size_t i = 0; i < 16; i++)  // parses bytes into a boolean array bit by bit
  {
    config_bit[i] = config_reg & (data_mask << i);
  }

  for (size_t i = 16; i > 0; i--)  // sends data to the controller leg from the array
  {
    /* 
         * For the love of god don't change a fucking thing here! 
         * Refactoring it, even changing gpio_set_level order, results in the panel to display garbage!
         * Potential electrical crosstalk!?
         */
    if ((i < latch_trigger_point) && (latch == true))  // for reg1, data latch on the last 4 bits when latch is true (on last send)
    {
      gpio_set_level((gpio_num_t)MBI_LAT, 1);
      gpio_set_level(MBI_G1, config_bit[i - 1]);
      gpio_set_level(MBI_G2, config_bit[i - 1]);
      gpio_set_level(MBI_G3, config_bit[i - 1]);
      gpio_set_level(MBI_G4, config_bit[i - 1]);

      gpio_set_level(MBI_R1, config_bit[i - 1]);
      gpio_set_level(MBI_R2, config_bit[i - 1]);
      gpio_set_level(MBI_R3, config_bit[i - 1]);
      gpio_set_level(MBI_R4, config_bit[i - 1]);

      gpio_set_level(MBI_B1, config_bit[i - 1]);
      gpio_set_level(MBI_B2, config_bit[i - 1]);
      gpio_set_level(MBI_B3, config_bit[i - 1]);
      gpio_set_level(MBI_B4, config_bit[i - 1]);

      mbi_clock(1);
      //     ESP_LOGD(TAG, "Sent config bit with latch HIGH");
    } else {
      gpio_set_level(MBI_G1, config_bit[i - 1]);
      gpio_set_level(MBI_G2, config_bit[i - 1]);
      gpio_set_level(MBI_G3, config_bit[i - 1]);
      gpio_set_level(MBI_G4, config_bit[i - 1]);

      gpio_set_level(MBI_R1, config_bit[i - 1]);
      gpio_set_level(MBI_R2, config_bit[i - 1]);
      gpio_set_level(MBI_R3, config_bit[i - 1]);
      gpio_set_level(MBI_R4, config_bit[i - 1]);

      gpio_set_level(MBI_B1, config_bit[i - 1]);
      gpio_set_level(MBI_B2, config_bit[i - 1]);
      gpio_set_level(MBI_B3, config_bit[i - 1]);
      gpio_set_level(MBI_B4, config_bit[i - 1]);

      mbi_clock(1);
    }
  }
  gpio_set_level((gpio_num_t)MBI_LAT, 0);

  gpio_set_level(MBI_G1, 0);
  gpio_set_level(MBI_G2, 0);
  gpio_set_level(MBI_G3, 0);
  gpio_set_level(MBI_G4, 0);


  gpio_set_level(MBI_R1, 0);
  gpio_set_level(MBI_R2, 0);
  gpio_set_level(MBI_R3, 0);
  gpio_set_level(MBI_R4, 0);

  gpio_set_level(MBI_B1, 0);
  gpio_set_level(MBI_B2, 0);
  gpio_set_level(MBI_B3, 0);
  gpio_set_level(MBI_B4, 0);

  // ESP_LOGD(TAG, "Completed MBI config sending. Latch is now low.");
}


/*

///////////////////////////
// SETTING A SINGLE PIN  //
///////////////////////////
// if "pin" is a compile time constant the if statement will be compiled out.
if(pin>31) {
    // set this pin by writing the bit for the pin
    // to the high bank
    GPIO.out1_w1ts.val = (1 << ((pin - 32)&31));
} else if(pin>-1) {
    // set the pin in the low bank
    GPIO.out_w1ts = (1 << (pin &31));
}
 
///////////////////////////
// CLEARING A SINGLE PIN //
///////////////////////////
// if "pin" is a compile time constant the if statement will be compiled out.
if(pin>31) {
    // clear this pin by writing the bit for the pin
    // to the high bank
    GPIO.out1_w1tc.val = (1 << ((pin - 32)&31));
} else if(pin>-1) {
    // clear the pin in the low bank
    GPIO.out_w1tc = (1 << (pin &31));
}
/////////////////////////
// SET/CLEAR MANY PINS //
/////////////////////////
 
To do this you simply combine the flags you want to set or clear into one 32 bit value
GPIO.out_w1ts = (1<<pin_d0)|(1<<pin_d1)||(1<<pin_d6); // set pin_d0, pin_d1, and pin_d6
// above assumes those pins are less than GPIO 32
// And the same with clearing them
 
/////////////////////////
// NOTES               //
/////////////////////////
 
// This can be too fast sometimes if you use it to control timing sensitive components.
// I workaround that by setting/clearing multiple times instead of just once
// 
// Even for a single pin this is much faster than digitalWrite for some reason
*/


/*sends data for 1 frame*/
void mbi_update_frame() 
{
  for (int i = 0; i < PANEL_SCAN_LINES*PANEL_MBI_RES_X*16; i++) 
  {

      if ( ((i+1) % 80) == 0 ) {
              gpio_set_level((gpio_num_t)MBI_LAT, 1);
      }

      // Shift out the First 1/4 of RGB panel - 20 rows
      gpio_set_level(MBI_G1, (greyscale_gpio_data[i]  & BIT_G1));
      gpio_set_level(MBI_B1, (greyscale_gpio_data[i]  & BIT_B1));
      gpio_set_level(MBI_R1, (greyscale_gpio_data[i]  & BIT_R1));

      // Second Quarter
      gpio_set_level(MBI_G2, (greyscale_gpio_data[i]  & BIT_G2));
      gpio_set_level(MBI_B2, (greyscale_gpio_data[i]  & BIT_B2));
      gpio_set_level(MBI_R2, (greyscale_gpio_data[i]  & BIT_R2));

      // Offset for next block of scan lines
      gpio_set_level(MBI_G3, (greyscale_gpio_data[i]  & BIT_G3));
      gpio_set_level(MBI_B3, (greyscale_gpio_data[i]  & BIT_B3));
      gpio_set_level(MBI_R3, (greyscale_gpio_data[i]  & BIT_R3));

      gpio_set_level(MBI_G4, (greyscale_gpio_data[i]  & BIT_G4));
      gpio_set_level(MBI_B4, (greyscale_gpio_data[i]  & BIT_B4));
      gpio_set_level(MBI_R4, (greyscale_gpio_data[i]  & BIT_R4));

      mbi_clock(1);

      if ( ((i+1) % 80) == 0 ) {
              gpio_set_level((gpio_num_t)MBI_LAT, 0);
      }

  }

  mbi_clock(50);    
}
