#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "sdkconfig.h"
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include "app_constants.hpp"
#include "lcd_dma_parallel16.hpp"
#include "spi_dma_tx_loop.h"
#include "Fastnoise.h"
//#include <FastLED.h> // Fastled doesn't compileon S3, fuck it. No pattern plazma for me.
//#include <Arduino.h>

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
const char *TAG       = "app_main";

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

#define gclk_multiplier_ON      1   // GCLK Multipler On - You MUST use exactly 257 clocks for each rowscan!
#define gclk_multiplier_OFF     0   // GCLK Multipler On - You MUST use exactly 513 clocks for each rowscan!
#define current_1               15  //ток на светодиоде
#define current_2               63  //ток на светодиоде
#define current_3               35  //ток на светодиоде
#define current_4               20  //LED current

#define clock_delay             0
#define pwm_hi                  65535  
#define pwm_lo                  0 


// Definition
void mbi_update_frame(bool configure_latches = true);

// D<A Data to send
Bus_Parallel16 dma_bus;

DMA_ATTR ESP32_GREY_DMA_STORAGE_TYPE *dma_grey_gpio_data;
size_t   dma_grey_buffer_size;

FastNoiseLite noise;

int simplexColorR = 0;
int simplexColorG = 209;
int simplexColorB = 255;

int simplexBrightness = -33;
float simplexContrast = 72;
float simplexScale = 5;
float simplexSpeed = 20;

#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif


// Like only 4 of the 8 bits get sent out in parallel...
DMA_ATTR uint16_t lcd_dma_test_payload[] = {
  0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,
  0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,
  0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,
  0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000,0xffff,0x0000
};


void lcd_send_test_payload() {
  
  dma_bus.send_stuff_once(lcd_dma_test_payload, sizeof(lcd_dma_test_payload), false);

}

void mbi_update_frame(bool configure_latches) {
    if (configure_latches)
    {
        int counter = 0;
        for (int row = 0; row < PANEL_SCAN_LINES; row++) {
          for (int chan = 0; chan < PANEL_MBI_LED_CHANS; chan++) {
            for (int ic = 0; ic < PANEL_MBI_CHAIN_LEN; ic++) { // number of chained ICs

                // data latch on the last bit, when sending the last byte set latch=1
                int latch = 0;
                if (ic == 4) { latch = 1; }  // latch on last channel / ic

                int bit_offset = 16;
                while (bit_offset > 0)  // shift out MSB first per the documentation.
                {
                  bit_offset--;  // start from 15

                  if (latch == 1 && bit_offset == 0) {
                    dma_grey_gpio_data[counter] |=  BIT_LAT;               
                  } 
                  counter++;             
                }

            }
          }
        }      
    }

     ESP_LOGD(TAG, "Sending greyscale data buffer out via LCD DMA.");
     dma_bus.send_stuff_once(dma_grey_gpio_data, dma_grey_buffer_size, true); // sending payload hence TRUE 
  
} // mbi_update_frame

/*
  Gray Scale Mode and Scan-type S-PWM
  MBI5153 provides a selectable 14-bit or 13-bit gray scale by setting the configuration register1 bit [7]. The default 
  value is set to ’0’ for 14-bit color depth. In 14-bit gray scale mode, users should still send 16-bit data with 2-bit ‘0’ in 
  LSB bits. For example, {14’h1234, 2’h0}. 
  MBI5153 has a smart S-PWM technology for scan type. With S-PWM, the total PWM cycles can be broken into MSB 
  (Most Significant Bits) and LSB (Least Significant Bits) of gray scale cycles. The MSB information can be broken 
  down into many refresh cycles to achieve overall same high bit resolution.

  The 16384 GCLKs (14-bit) PWM cycle of MBI5052/53 is divided into 32 sections, each section has 512 GCLKs.
*/
void mbi_set_pixel(uint8_t x, uint8_t  y, uint8_t r_data, uint8_t g_data, uint8_t b_data) {

  if ( x >= PANEL_PHY_RES_X || y >= PANEL_PHY_RES_Y) {
    return;
  }

  x += 2; // offset for missing pixels on the left


  //14-bit resolution = 16,384
  // 8-bit resolutoin = 255
  // ... that's 64 times larger
  uint16_t g_14bit_data = (g_data*64) << 2;  
  uint16_t b_14bit_data = (b_data*64) << 2;    
  uint16_t r_14bit_data = (r_data*64) << 2; // could just bit shift by << 6  in total instead


  ESP_LOGD(TAG, "Converted 14bit colour value r: %d,  g: %d,  b: %d", g_14bit_data, g_14bit_data, b_14bit_data);  


  // x and y positions start from 0, so 0-78 are valid values only
  uint16_t g_gpio_bitmask = BIT_G1; // bit 0
  uint16_t b_gpio_bitmask = BIT_B1; // bit 1
  uint16_t r_gpio_bitmask = BIT_R1; // bit 2

  uint16_t _colourbitclear = BIT_RGB1_CLR, _colourbitoffset = 0;

  _colourbitoffset = (y/PANEL_SCAN_LINES) * 3; // three is an important bit
  _colourbitclear  = ~(0b111 << _colourbitoffset); // invert

  g_gpio_bitmask = g_gpio_bitmask << _colourbitoffset;
  b_gpio_bitmask = b_gpio_bitmask << _colourbitoffset;
  r_gpio_bitmask = r_gpio_bitmask << _colourbitoffset;  

  /*
    if (y < 20) {
    } else if (y < 40) {
      b_gpio_bitmask = BIT_B2;
      g_gpio_bitmask = BIT_G2;
      r_gpio_bitmask = BIT_R2;      
    } else if (y < 60)  {
      b_gpio_bitmask = BIT_B3;
      g_gpio_bitmask = BIT_G3;
      r_gpio_bitmask = BIT_R3;      
    } else {
      b_gpio_bitmask = BIT_B4;
      g_gpio_bitmask = BIT_G4;
      r_gpio_bitmask = BIT_R4;      
    }
    */

    // Row offset + channel offset + individual IC LED offset
    int y_normalised  = y % PANEL_SCAN_LINES; // Only have 20 rows of data... 
    int bit_start_pos = (1280*y_normalised)+((x%16)*80)+((x/16)*16);

    int bit_offset = 16;
    while (bit_offset > 0)  // shift out MSB first per the documentation.
    {
      bit_offset--;  // start from 15

      uint16_t mask = 1 << bit_offset;

      dma_grey_gpio_data[bit_start_pos] &= _colourbitclear; // clear relevant rgb bits

      if ( g_14bit_data & mask ) { 
        dma_grey_gpio_data[bit_start_pos] |=  g_gpio_bitmask;
      }

      if ( b_14bit_data & mask ) { 
        dma_grey_gpio_data[bit_start_pos] |=  b_gpio_bitmask;
      }

      if ( r_14bit_data & mask ) { 
        dma_grey_gpio_data[bit_start_pos] |=  r_gpio_bitmask;
      }      

      ESP_LOGD(TAG, "Setting dma_grey_gpio_data from bit_start_pos %d. Value %d", bit_start_pos, dma_grey_gpio_data[bit_start_pos]);  

      bit_start_pos++;
    }

} // mbi_greyscale_data_set_pixel


/*pre-activation command - sent before sending configuration register data*/
void mbi_pre_active_dma() {
  
  ESP_LOGD(TAG, "Send MBI Pre-Active.");

  int payload_length = 0; // length in parallel CLOCK cycles
  for (int i = 0; i <14; i++) {
    dma_grey_gpio_data[payload_length] = BIT_LAT;
    payload_length++;
  }
  

  /* LE/LAT should be low for any rising edge of DCLK */
  for (int i = 0; i <2; i++) {
    dma_grey_gpio_data[payload_length] = 0x00;
    payload_length++;
  }

  dma_bus.send_stuff_once(dma_grey_gpio_data, payload_length*sizeof(ESP32_GREY_DMA_STORAGE_TYPE), false);

  /*
    gpio_set_level((gpio_num_t)MBI_LAT, 1);
    mbi_clock(14);
    gpio_set_level((gpio_num_t)MBI_LAT, 0);
    mbi_clock(2);
  */

}

/*vertical sync - updates frame data on outputs, used in conjunction with vertical scan*/
// 9. GCLK must keep at low level more than 7ns before MBI5152 receives the Vsync signa
// 7ns = 142 Mhz clock speed...
/* After the last data latch command, it needs at least 50 GCLKs to read the gray scale data into internal display 
buffer before the Vsync command comes. And display is updated immediately until MBI5051/52/53 receives 
the Vsync signal (high pulse of LE pin is sampled by 3-DCLK rising edges), as figure 6 shows. 
*/
void mbi_v_sync_dma() {
  ESP_LOGD(TAG, "Send MBI Vert Sync.");

  int payload_length = 0;
  for (int i = 0; i <3; i++) {
    dma_grey_gpio_data[payload_length] = BIT_LAT;
    payload_length++;

  }

  dma_grey_gpio_data[payload_length] = 0x00;
  payload_length++;

  dma_bus.send_stuff_once(dma_grey_gpio_data, payload_length*sizeof(ESP32_GREY_DMA_STORAGE_TYPE), false);

  /*
    gpio_set_level((gpio_num_t)MBI_LAT, 1);
    mbi_clock(2);
    gpio_set_level((gpio_num_t)MBI_LAT, 0);
  */  
}

/*soft reset*/
void mbi_soft_reset_dma() {
  /* “Software reset” command makes MBI5153 go back to the initial state except configuration register value. After this 
  command is received, the output channels will be turned off and will display again with last gray-scale value after 
  new “Vsync” command is received.
  */
  ESP_LOGD(TAG, "Send MBI Soft Reset.");

  int payload_length = 0;
  for (int i = 0; i <10; i++) {
    dma_grey_gpio_data[payload_length] = BIT_LAT;
    payload_length++;

  }

  dma_grey_gpio_data[payload_length] = 0x00;
  payload_length++;

  dma_bus.send_stuff_once(dma_grey_gpio_data, payload_length*sizeof(ESP32_GREY_DMA_STORAGE_TYPE), false);

  /*
    gpio_set_level((gpio_num_t)MBI_LAT, 1);
    mbi_clock(10);
    gpio_set_level((gpio_num_t)MBI_LAT, 0);
  */

}

static int  config_payload_bit_pos = 0;
void mbi_send_config_reg1_dma() {

   // Step 1) Send configuration for 
   uint16_t config_reg1_val = 0;

   int ghost_elimination    = ghost_elimination_ON;
   int line_num             = PANEL_SCAN_LINES-1;
   int gray_scale           = gray_scale_14; 
   int gclk_multiplier      = gclk_multiplier_OFF;
   int current              = current_1; // change as required by channel

  //Documentation says set bits E and F of Config1 Reg to 1
  config_reg1_val = (config_reg1_val | (ghost_elimination << 14) | (line_num << 8) | (gray_scale << 7) | (gclk_multiplier << 6) | (current));

  unsigned int dma_output_pos = 0;
  for (int i = 0; i < PANEL_MBI_CHAIN_LEN; i++) {
    mbi_set_config_dma(dma_output_pos, config_reg1_val, (i == (PANEL_MBI_CHAIN_LEN - 1)), false);  // on last mbi chip, latch  
  }

  dma_bus.send_stuff_once(dma_grey_gpio_data, dma_output_pos*sizeof(ESP32_GREY_DMA_STORAGE_TYPE), false);

}

/*настройка регистра конфигурации*/
void mbi_send_config_reg2_dma() {
  // F E D C B A 9 8 7 6 5 4 3 2 1 0
  //uint16_t config_reg2_val = 0b0001000000010000; // default value apparently

  uint16_t config_reg2_val = 0b1001000000011110; // removes ghosting except for red
  unsigned int dma_output_pos = 0;

  for (int i = 0; i < PANEL_MBI_CHAIN_LEN; i++) {
    mbi_set_config_dma(dma_output_pos, config_reg2_val, (i == (PANEL_MBI_CHAIN_LEN - 1)), true);  // on last mbi chip, latch  
  }

  dma_bus.send_stuff_once(dma_grey_gpio_data, dma_output_pos*sizeof(ESP32_GREY_DMA_STORAGE_TYPE), false);

}


/*отправка регистра конфигурации*/
/* Need to set config for all MBI input on the panel at once as they all share the same latch pin */
void mbi_set_config_dma(unsigned int &dma_output_pos, uint16_t config_reg, bool latch, bool reg2) {
  //  ESP_LOGD(TAG, "Send MBI config");

  // Number of DCLK Rising Edge when LE is asserted 
  // Write Configuration 1 = 4
  // Write Configuration 2 = 8
  int latch_trigger_point = (reg2) ? 8:4; 

  for (int bit = 15; bit >= 0; bit--) {
    
      int bitval = ((config_reg >> bit) & 1 );

      uint16_t mbi_rgb_sdi_val = 0;
      if (bitval) {
          mbi_rgb_sdi_val = BIT_ALL_RGB; // all <BI colour channels get the same config for now.
      }

      if ( (bit < latch_trigger_point) && (latch == true) )  { // for reg1, data latch on the last 4 bits when latch is true (on last send)

          //std::cout << "Latch." << std::endl;   
          mbi_rgb_sdi_val |= BIT_LAT;
      }     
      
      dma_grey_gpio_data[dma_output_pos++] =  mbi_rgb_sdi_val;

  } // iterate through bits

} // mbi_send_config_dma


/************************************************************************/
float tx, nx, p;
float ty, ny, py;
float rot, rotx, roty, rotz, rotxx, rotyy, rotzz, rotxxx, rotyyy, rotzzz;
int i; //0 to 360
int fl, scale; //focal length
int wireframe[12][2];

int originx = 40;
int originy = 40; //32

int front_depth = 20;
int back_depth = -20;

//Store cube vertices
int cube_vertex[8][3] = {
 { -20, -20, front_depth},
 {20, -20, front_depth},
 {20, 20, front_depth},
 { -20, 20, front_depth},
 { -20, -20, back_depth},
 {20, -20, back_depth},
 {20, 20, back_depth},
 { -20, 20, back_depth}
};

int fd = 0; //0=orthographic


void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1  ) {
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }

  if (x0 > x1) {
    _swap_int16_t(x0, x1);
    _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0 <= x1; x0++) {
    if (steep) {
      mbi_set_pixel(y0, x0, 0,255,255);
    } else {
      mbi_set_pixel(x0, y0, 255,0,255);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}


void draw_vertices(void)
{
 mbi_set_pixel (rotxxx, rotyyy, 255, 0, 0);
}

void draw_wireframe(void)
{
 writeLine(wireframe[0][0], wireframe[0][1], wireframe[1][0], wireframe[1][1]);
 writeLine(wireframe[1][0], wireframe[1][1], wireframe[2][0], wireframe[2][1]);
 writeLine(wireframe[2][0], wireframe[2][1], wireframe[3][0], wireframe[3][1]);
 writeLine(wireframe[3][0], wireframe[3][1], wireframe[0][0], wireframe[0][1]);

//cross face above
 writeLine(wireframe[1][0], wireframe[1][1], wireframe[3][0], wireframe[3][1]);
 writeLine(wireframe[0][0], wireframe[0][1], wireframe[2][0], wireframe[2][1]);

 writeLine(wireframe[4][0], wireframe[4][1], wireframe[5][0], wireframe[5][1]);
 writeLine(wireframe[5][0], wireframe[5][1], wireframe[6][0], wireframe[6][1]);
 writeLine(wireframe[6][0], wireframe[6][1], wireframe[7][0], wireframe[7][1]);
 writeLine(wireframe[7][0], wireframe[7][1], wireframe[4][0], wireframe[4][1]);
 
 writeLine(wireframe[0][0], wireframe[0][1], wireframe[4][0], wireframe[4][1]);
 writeLine(wireframe[1][0], wireframe[1][1], wireframe[5][0], wireframe[5][1]);
 writeLine(wireframe[2][0], wireframe[2][1], wireframe[6][0], wireframe[6][1]);
 writeLine(wireframe[3][0], wireframe[3][1], wireframe[7][0], wireframe[7][1]);
}


/********************************************************************/

// Start the App
void setup(void)
{
    Serial.begin(115200);
    delay(2);
    Serial.println("Starting....");
    esp_task_wdt_deinit();

    // MBI_SRCLK used for testing
    esp_rom_gpio_pad_select_gpio(MBI_SRCLK); // scan row clock in?
    gpio_set_direction(MBI_SRCLK, GPIO_MODE_OUTPUT);         
    gpio_set_level(MBI_SRCLK, 0); // to aid with debugging   


    ESP_LOGD(TAG, "Allocation DMA Memory And Buffer"); 

    // Step 1) Allocate raw buffer space for MBI5153 greyscale / pixel memory
    dma_grey_buffer_size = sizeof(ESP32_GREY_DMA_STORAGE_TYPE)*((PANEL_SCAN_LINES*PANEL_MBI_RES_X*16)+100); // add some extract blank data at the end for time to latch if we're updating the greyscale buffers 
    ESP_LOGD(TAG, "Allocating greyscale DMA memory buffer. Size of memory required: %lu bytes.", dma_grey_buffer_size);  

    // Malloc Greyscale / Command DMA Memory
    dma_grey_gpio_data = (ESP32_GREY_DMA_STORAGE_TYPE *)heap_caps_malloc(dma_grey_buffer_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);  
    assert(dma_grey_gpio_data != nullptr);

    // Fill with zeros to start with
    memset(dma_grey_gpio_data, 0, dma_grey_buffer_size);    

   // Setup DMA and Output to GPIO
    auto bus_cfg = dma_bus.config(); 
    bus_cfg.pin_wr      = MBI_DCLK; // DCLK Pin
    bus_cfg.invert_pclk = false;
    bus_cfg.pin_d0  = MBI_G1; 
    bus_cfg.pin_d1  = MBI_B1;
    bus_cfg.pin_d2  = MBI_R1;
    bus_cfg.pin_d3  = MBI_G2;
    bus_cfg.pin_d4  = MBI_B2;
    bus_cfg.pin_d5  = MBI_R2;
    bus_cfg.pin_d6  = MBI_G3;
    bus_cfg.pin_d7  = MBI_B3;
    bus_cfg.pin_d8  = MBI_R3;    
    bus_cfg.pin_d9  = MBI_G4;
    bus_cfg.pin_d10 = MBI_B4;
    bus_cfg.pin_d11 = MBI_R4;    
    bus_cfg.pin_d12 = MBI_LAT;  // Latch      
    bus_cfg.pin_d13 = -1;       // DCLK potentially if we need to manually generate    
    bus_cfg.pin_d14 = -1;      
    bus_cfg.pin_d15 = -1;                

    dma_bus.config(bus_cfg);
    dma_bus.setup_lcd_dma_periph();

    // Send blank message payload just to get DMA descriptors setup.
    //mbi_update_frame(true);

    // CURRENT STATUS - LOOPS FOREVER, BUT WOORKING!
    //dma_bus.send_stuff_once(lcd_dma_test_payload, sizeof(lcd_dma_test_payload));

    //gpio_set_level(MBI_SRCLK,   1); // to aid with debugging    

    // Start GCLK via SPI
    spi_setup();
    spi_transfer_loop_start(); // start GCLK + Adress toggling

    // MBI Step 1) Set key registers, such as number of rows
    mbi_soft_reset_dma(); // 10 clocks
    mbi_pre_active_dma(); // 14 clocks
    mbi_send_config_reg1_dma();

    // MBI Step 2) Some other register 2 hack to reduce ghosting.
    mbi_pre_active_dma();
    mbi_send_config_reg2_dma();  

    // MBI Step 3) Clean out any crap in the greyscale buffer
    mbi_soft_reset_dma();  // 10 clocks
    

    memset(dma_grey_gpio_data, 0, dma_grey_buffer_size);    // accidenty display configuration registers id we don't clear th
    //mbi_set_pixel(0,0, 255,255,255);
    for (int p = 0; p < 78; p++)
    {
      mbi_set_pixel(p,p, 255,255,255);
      mbi_set_pixel((78-p),p, 255,255,255);
    }

    mbi_update_frame(true);    

    
    
    //ESP_LOGD(TAG, "Sending greyscale data buffer out via LCD DMA.");
    //dma_bus.send_stuff_once(dma_grey_gpio_data, counter2*sizeof(uint16_t), true); // sending payload hence TRUE 

    spi_transfer_loop_stop();
    mbi_v_sync_dma();
    spi_transfer_loop_restart();

    
    noise.SetNoiseType(FastNoiseLite::NoiseType_OpenSimplex2S);
    noise.SetRotationType3D(FastNoiseLite::RotationType3D_ImproveXYPlanes);
    noise.SetFrequency(.0004);
    //   noise.SetFractalType(FastNoiseLite::FractalType_FBm);
    //   noise.SetFractalLacunarity(2.7);
    //   noise.SetFractalOctaves(6);
    //   noise.SetFractalGain(0.1);
    //   noise.SetFractalLacunarity(2.7);
    //   noise.SetFractalWeightedStrength(.1);
    noise.SetDomainWarpType(FastNoiseLite::DomainWarpType_OpenSimplex2);
    noise.SetDomainWarpAmp(240);

    delay(500);
 
    

}

//extern volatile int transfer_count;

unsigned long last_count        = 20000;
unsigned long last_reg_update   =  10000;
unsigned long frames = 0;
void loop() {

    if ((millis() - last_count) > 1000) {
        Serial.print("FPS: ");
        Serial.println(frames, DEC);
        frames = 0;
        last_count = millis();

        //Serial.print("Interrupt count: ");
        //Serial.println(transfer_count, DEC);
        
    }

    // Periodically reset reg1 update incase of corruption
    // or the panel power being reset (and ESP32 still running)
    if (  (millis() - last_reg_update)  > 10000   ) {
    
      mbi_pre_active_dma(); // 14 clocks 
      mbi_send_config_reg1_dma();

      last_reg_update = millis();

    }

    
    //picture loop
//     for (int angle = 0; angle <= 360; angle = angle + 3) {
//    
//          for (int i = 0; i < 8; i++) {
//
//         memset(dma_grey_gpio_data, 0, dma_grey_buffer_size);    // accidenty display configuration registers id we don't clear th
//    
//        rot = angle * 0.0174532; //0.0174532 = one degree
//    //rotateY
//        rotz = cube_vertex[i][2] * cos(rot) - cube_vertex[i][0] * sin(rot);
//        rotx = cube_vertex[i][2] * sin(rot) + cube_vertex[i][0] * cos(rot);
//        roty = cube_vertex[i][1];
//    //rotateX
//        rotyy = roty * cos(rot) - rotz * sin(rot);
//        rotzz = roty * sin(rot) + rotz * cos(rot);
//        rotxx = rotx;
//    //rotateZ
//        rotxxx = rotxx * cos(rot) - rotyy * sin(rot);
//        rotyyy = rotxx * sin(rot) + rotyy * cos(rot);
//        rotzzz = rotzz;
//    
//    //orthographic projection
//        rotxxx = rotxxx + originx;
//        rotyyy = rotyyy + originy;
//    
//    //store new vertices values for wireframe drawing
//        wireframe[i][0] = rotxxx;
//        wireframe[i][1] = rotyyy;
//        wireframe[i][2] = rotzzz;
//    
//        draw_vertices();
//       }
//    
//       draw_wireframe();
        
        memset(dma_grey_gpio_data, 0, dma_grey_buffer_size);

        for (int i = 0; i < 80; i++) {
         for (int j = 0; j < 80; j++) {
           int col = int((1 + noise.GetNoise(j * simplexScale * 10, i * simplexScale * 10, float(millis() * simplexSpeed / 50))) * 127);
           col += simplexBrightness;
           col = constrain(col, 0, 255);
           float contrastFactor = (259 * (simplexContrast + 255)) / (255 * (259 - simplexContrast));
           col = contrastFactor * (col - 128) + 128;
           col = constrain(col, 0, 255);

            mbi_set_pixel(j, i, uint8_t(col * simplexColorR / 255.0f),uint8_t(col * simplexColorG / 255.0f),uint8_t(col * simplexColorB / 255.0f));
         }
       }
        
        mbi_update_frame(true);    
        
        spi_transfer_loop_stop();
        mbi_v_sync_dma();
        spi_transfer_loop_restart();  
        frames++;     


 }
