#include "ESP32-HUB75-MatrixPanel-I2S-DMA.h"



bool MatrixPanel_I2S_DMA::begin()
{
  // As DMA buffers are dynamically allocated, we must allocated in begin() 
  // Ref: https://github.com/espressif/arduino-esp32/issues/831
  
  /* 
    To update the panel of 80 pixels over 20 lines, each MBI chain (there are 12 seperate chains) needs 25600 bits
    (with 4 scan section all updated in parallel by 12bits of the 21 parallel bites using the 4 x 3 rgb pwm chils, so 80px height done at once)

     Each parallel bit pulse uses a 4byte uint32_t, so 25,600 x 4 bytes = 102,400 bytes    

     The most important bit is that 25,600 bits must be sent.

     ---

     However, to ensure that the PWM chips iterate correctly through all 20 lines, we need to send 20*513 = 10,260 bits

     If we do this over three cycles for each GCLK, that's 20*513*2 = 20,520 bits ()
   */

    
  // Allocate greyscale framebuffer WITH GCLKS as well!
  // Step 1) Allocate raw buffer space for MBI5153 greyscale / pixel memory
  size_t dma_grey_buffer_length = (PANEL_SCAN_LINES*PANEL_MBI_RES_X*16); // add some extract blank data at the end for time to latch if we're updating the greyscale buffers 
  output_bits_greyscale = new ParallelDataOutputStruct(dma_grey_buffer_length, false);  

/*
  // Step 2) Allocate GCLKS only buffer - To keep shit showing on screen by stimulating the PWM
  int glck_length = PANEL_MBI_RES_X*gclk_multiplier_OFF_clks*2;
  output_fb[2] = new ParallelDataOutputStruct(glck_length, true);

  // Allocate greyscale framebuffer C - C stands for command
  // General command data shit to send out in parallel
  output_fb[3] = new ParallelDataOutputStruct(512, false);  
*/

  int fb_descriptors_num = output_bits_greyscale->get_dma_lldesc_required();


  dma_bus.enable_double_dma_desc();
  dma_bus.allocate_dma_desc_memory(fb_descriptors_num);

  // use fb_descriptors_num to iterate through
  for (int dma_desc_idx = 0; dma_desc_idx < fb_descriptors_num; dma_desc_idx++) 
  {
      int num_uint32_per_dmadesc = DMA_MAX / sizeof(ESP32_I2S_DMA_STORAGE_TYPE);
      int dma_startpos    = dma_desc_idx*num_uint32_per_dmadesc;

      int bytes_left      = output_bits_greyscale->getByteSize() - (dma_startpos*sizeof(ESP32_I2S_DMA_STORAGE_TYPE));
      int dmachunklen     = (bytes_left > DMA_MAX) ? DMA_MAX:bytes_left;

      dma_bus.create_dma_desc_link(&output_bits_greyscale->data[dma_startpos], dmachunklen, false);
      ESP_LOGD("I2S-DMA begin()", "Configured dmadesc at pos %d of %d, memory location %08x, pointing to chunk of size %d bytes with %d bytes left.", dma_desc_idx, fb_descriptors_num, (uintptr_t)&output_bits_greyscale->data[dma_startpos], (int) dmachunklen, bytes_left);      
  }


  // Setup DMA and Output to GPIO
  auto bus_cfg = dma_bus.config(); 

  //bus_cfg.bus_freq    = 4000000; //m_cfg.i2sspeed;
  bus_cfg.pin_wr      = I2S_WR_CLOCK;
  bus_cfg.invert_pclk = false;

  // Do not change this order at all.
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

  dma_bus.config(bus_cfg);

  dma_bus.init();

  dma_bus.dma_transfer_start();

  // i2s_parallel_send_dma(ESP32_I2S_DEVICE, &dmadesc_a[0]);
  ESP_LOGI("I2S-DMA", "DMA setup completed");

  return true;

} // end initMatrixDMABuff



void IRAM_ATTR MatrixPanel_I2S_DMA::updateMatrixDMABuffer(uint16_t x_coord, uint16_t y_coord, uint8_t red, uint8_t green, uint8_t blue)
{



} // updateMatrixDMABuffer (specific co-ords change)
