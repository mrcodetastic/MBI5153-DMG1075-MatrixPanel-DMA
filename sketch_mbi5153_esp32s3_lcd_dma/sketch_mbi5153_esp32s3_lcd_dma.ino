#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "sdkconfig.h"
#include <esp_log.h>
#include "esp_task_wdt.h"
#include "ewaste_MBI5153.hpp"
#include "mbi_gpio.h"
#include "app_constants.hpp"
#include "gdma_lcd_parallel16.hpp"
#include "gclk_data.h"

static const char *TAG  = "app_main";
volatile int refresh    = 0;
volatile int image      = 0;

// For GCLK
Bus_Parallel16 dma_bus;

uint8_t *dma_gpio_data;
size_t   dma_gpio_size;

// For Greyscale
uint16_t *greyscale_gpio_data; 
size_t    greyscale_buffer_size;


bool setup_dma_and_pixel_buffer() 
{ 

    // Step 1) Allocate raw buffer space for MBI5153 greyscale / pixel memory (25.6kB)
    greyscale_buffer_size = sizeof(uint16_t)*(PANEL_SCAN_LINES*PANEL_MBI_RES_X*16); // add some extract blank data at the end for time to latch if we're updating the greyscale buffers 
    ESP_LOGD(TAG, "Allocating greyscale DMA memory buffer. Size of memory required: %lu bytes.", greyscale_buffer_size);  

    // Malloc Greyscale / Command DMA Memory
    greyscale_gpio_data = (uint16_t *)heap_caps_malloc(greyscale_buffer_size, MALLOC_CAP_INTERNAL);  
    assert(greyscale_gpio_data != nullptr);

    // Fill with zeros to start with
    memset(greyscale_gpio_data, 0, greyscale_buffer_size);    


    // Step 2) Allocate DMA member for GCLK signals (~20kB)
    dma_gpio_size        = sizeof(gclk_addr_data_nodt);
    ESP_LOGD("I2S-DMA", "Size of DMA data we need to use to drive is %u.", dma_gpio_size);  
    
    // Defined in main.hpp
    dma_gpio_data = (uint8_t *)heap_caps_malloc(dma_gpio_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

    assert(dma_gpio_data != nullptr);

    ESP_LOGD("I2S-DMA", "Copying bootstrap DMA data across.");                
    memcpy(dma_gpio_data, gclk_addr_data_nodt, dma_gpio_size);    // Using the same data for now.

    // Calculate DMA descriptors required
    int dma_descriptors_required = (dma_gpio_size/DMA_MAX) + (dma_gpio_size % DMA_MAX != 0);
    int last_dma_packet_size     = (dma_gpio_size % DMA_MAX == 0) ? DMA_MAX:(dma_gpio_size % DMA_MAX);
        
    dma_bus.allocate_dma_desc_memory(dma_descriptors_required);
    ESP_LOGI("I2S-DMA", "%d DMA descriptors required for cover buffer data.", dma_descriptors_required);    

    // Link up DMA descriptors chain to DMA data.
    int dma_buffer_offset = 0;
    for (int dma_desc = 0; dma_desc < dma_descriptors_required-1; dma_desc++)
    {
        dma_bus.create_dma_desc_link(&dma_gpio_data[dma_buffer_offset], DMA_MAX, false);
        dma_buffer_offset += DMA_MAX;
    }
    dma_bus.create_dma_desc_link(&dma_gpio_data[dma_buffer_offset], last_dma_packet_size, false);
    
    
    // Setup DMA and Output to GPIO
    auto bus_cfg = dma_bus.config(); 

    bus_cfg.pin_wr      = MBI_SPARE; // must be allocated a clock pin. even if not used
    bus_cfg.invert_pclk = false;

    bus_cfg.pin_d0 = MBI_GCLK; // 
    bus_cfg.pin_d1 = ADDR_A_PIN;
    bus_cfg.pin_d2 = ADDR_B_PIN;
    bus_cfg.pin_d3 = ADDR_C_PIN;
    bus_cfg.pin_d4 = ADDR_D_PIN;
    bus_cfg.pin_d5 = ADDR_E_PIN;
    bus_cfg.pin_d6 = -1;
    bus_cfg.pin_d7 = -1;

    dma_bus.config(bus_cfg);

    dma_bus.init();

    return true;
}


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

    // Row offset + channel offset + individual IC LED offset
    int y_normalised  = y % PANEL_SCAN_LINES; // Only have 20 rows of data... 
    int bit_start_pos = (1280*y_normalised)+((x%16)*80)+((x/16)*16);

    int bit_offset = 16;
    while (bit_offset > 0)  // shift out MSB first per the documentation.
    {
      bit_offset--;  // start from 15

      uint16_t mask = 1 << bit_offset;

      greyscale_gpio_data[bit_start_pos] &= _colourbitclear; // clear relevant rgb bits

      if ( g_14bit_data & mask ) { 
        greyscale_gpio_data[bit_start_pos] |=  g_gpio_bitmask;
      }

      if ( b_14bit_data & mask ) { 
        greyscale_gpio_data[bit_start_pos] |=  b_gpio_bitmask;
      }

      if ( r_14bit_data & mask ) { 
        greyscale_gpio_data[bit_start_pos] |=  r_gpio_bitmask;
      }      

      ESP_LOGV(TAG, "Setting greyscale_gpio_data from bit_start_pos %d. Value %d", bit_start_pos, greyscale_gpio_data[bit_start_pos]);  

      bit_start_pos++;
    }

} // mbi_greyscale_data_set_pixel




/************************************************************************/



#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif



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
    esp_task_wdt_deinit();

    Serial.begin(112500);
       
    setup_gpio_dir();
    setup_gpio_output();    
    setup_dma_and_pixel_buffer(); 

    dma_bus.dma_transfer_start();  // start gclk + addr toggle

    mbi_soft_reset();    
  
    // Configure register 1    
    mbi_pre_active(); // must      
    mbi_configuration_1();  

    // Configure register 2 - Additional ghosting fix.
    mbi_pre_active(); // must          
    mbi_configuration_2();   

    mbi_soft_reset();        

}

unsigned long last_count        = 20000;
void loop()
{

/*
    // Clear greyscale buffer
    memset(greyscale_gpio_data, 0, greyscale_buffer_size);  

    for (int p = 0; p < 78; p++) {
      mbi_set_pixel(p,p, 255,255,255);
      mbi_set_pixel((78-p),p, 255,255,255);
    }

    mbi_update_frame();    
    dma_bus.dma_transfer_stop();          

    delay(1);     
    mbi_v_sync(); 
    delay(1);             
    dma_bus.dma_transfer_start();


    return;    
*/

    
    //picture loop
     for (int angle = 0; angle <= 360; angle = angle + 3) {
        for (int i = 0; i < 8; i++) {

          rot = angle * 0.0174532; //0.0174532 = one degree
          //rotateY
          rotz = cube_vertex[i][2] * cos(rot) - cube_vertex[i][0] * sin(rot);
          rotx = cube_vertex[i][2] * sin(rot) + cube_vertex[i][0] * cos(rot);
          roty = cube_vertex[i][1];
          //rotateX
          rotyy = roty * cos(rot) - rotz * sin(rot);
          rotzz = roty * sin(rot) + rotz * cos(rot);
          rotxx = rotx;
          //rotateZ
          rotxxx = rotxx * cos(rot) - rotyy * sin(rot);
          rotyyy = rotxx * sin(rot) + rotyy * cos(rot);
          rotzzz = rotzz;

          //orthographic projection
          rotxxx = rotxxx + originx;
          rotyyy = rotyyy + originy;

          //store new vertices values for wireframe drawing
          wireframe[i][0] = rotxxx;
          wireframe[i][1] = rotyyy;
          wireframe[i][2] = rotzzz;

          draw_vertices();

       }
    
        draw_wireframe();
    
    
        mbi_update_frame();    

        dma_bus.dma_transfer_stop();          

        /* Need to put a delay in for the moment as when we call dma_transfer_stop() the DMA transfer doesn't actually stop immediately
          * and continues in an async manner in the background. This causes a stuffup / corruption of pixel data around the time of the immediate vsync 
          *
          * Not sure what SRCLK does either, but toggling it high and then low around time of greyscale data transfer stops visible noise showing.
          */

        // Clear greyscale buffer
        memset(greyscale_gpio_data, 0, greyscale_buffer_size);  

       // delay(2);     
        mbi_v_sync(); 
        delay(2);             
    
        dma_bus.dma_transfer_start();

     }


} // loop()

