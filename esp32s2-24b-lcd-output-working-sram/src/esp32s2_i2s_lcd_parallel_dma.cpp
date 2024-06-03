#include <sdkconfig.h>

#if !defined (CONFIG_IDF_TARGET_ESP32S2)
  #pragma error "Designed only for ESP32-S2"
#endif


#include "esp32s2_i2s_lcd_parallel_dma.hpp"


#include <driver/gpio.h>
#if (ESP_IDF_VERSION_MAJOR == 5)
#include <esp_private/periph_ctrl.h>
#else
#include <driver/periph_ctrl.h>
#endif
#include <soc/gpio_sig_map.h>
#include <soc/i2s_periph.h> //includes struct and reg


#if defined (ARDUINO_ARCH_ESP32)
#include <Arduino.h>
#endif

#include <esp_err.h>
#include <esp_log.h>

// Get CPU freq function.
#include <soc/rtc.h>

  static const char *TAG = "edma_lcd_test";

  extern DMA_DATA_TYPE *global_buffer;
  extern lldesc_t *dma_ll;


  // Static
  i2s_dev_t* getDev() {
      return &I2S0;
  }

  // Static
  inline void _gpio_pin_init(int pin)
  {
    if (pin >= 0)
    {
      gpio_pad_select_gpio(pin);
      gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
      gpio_set_drive_capability((gpio_num_t)pin, (gpio_drive_cap_t)3);      // esp32s3 as well?
    }
  }

  // esp32 s2 only
  inline int i2s_parallel_get_memory_width(int port, int width) {
    switch(width) {
    case 8:
      return 1;
    case 16:
      return 2;
    case 24:
      return 4;
    default:
      return -ESP_ERR_INVALID_ARG;
    }
  }


  static lldesc_t * allocate_dma_descriptors_gb(uint32_t count, uint16_t payload_size, DMA_DATA_TYPE *buffer)
  {
      lldesc_t *dma = (lldesc_t *)heap_caps_malloc(count * sizeof(lldesc_t), MALLOC_CAP_DMA);
      if (dma == NULL) {
          ESP_LOGE("allocate_dma_descriptors()", "Could not allocate lldesc_t memory.");            
          return dma;
      }

      int n = 0;
      while (n < count) 
      {

          int dmachunklen = payload_size;
          if (dmachunklen > DMA_MAX) {
              dmachunklen = DMA_MAX;
          }

          dma[n].size = dmachunklen;
          dma[n].length = dmachunklen;

          dma[n].sosf = 0;
          dma[n].eof = 0;
          dma[n].owner = 1;     // 1 = dma   

          //dma[n].buf = (buffer);
          dma[n].buf = (uint8_t*)(buffer);

          ESP_LOGD("allocate_dma_descriptors()", "Linking to payload buff at memory location 0x%08X.", (uintptr_t)dma[n].buf);                     

          payload_size -= dmachunklen;        
          buffer += dmachunklen;  

          if ( n == (count-1) ) { // last element
            dma[n].empty = (uintptr_t)&dma[0];
            ESP_LOGD("allocate_dma_descriptors()", "Linking lldesc_t  pos %d back to pos 0", n);      

          } else {
            dma[n].empty = (uintptr_t)&dma[(n  + 1) % count];
            ESP_LOGD("allocate_dma_descriptors()", "Linking lldesc_t  pos %d to pos %d at memory location %08x.", n, (n+1), (uintptr_t)&dma[(n + 1) % count]);      
          }
            ESP_LOGD("allocate_dma_descriptors()", "Chunk len %d.", dmachunklen);          


        n++;


      }

      return dma;
  }


  static int ll_cam_get_dma_align()
  {
      ESP_LOGD("ll_cam_get_dma_align()", "ext_mem_bk_size val is %d", I2S0.lc_conf.ext_mem_bk_size);         

      //return 16; //16 << I2S0.lc_conf.ext_mem_bk_size;   
      //return 64;//16 << I2S0.lc_conf.ext_mem_bk_size;   
      //return 1024; //16 << I2S0.lc_conf.ext_mem_bk_size;    

      //return 32;
      //return 64;
      return 16;
  }


  static int ll_desc_get_required_num(uint32_t bytes_len)
  {
      int ll_desc_required = (bytes_len + DMA_MAX - 1) / DMA_MAX;
      ESP_LOGD("ll_desc_get_required_num()", "Gunna need %d dma lldesc's", ll_desc_required);      

      return ll_desc_required;
  }


  esp_err_t i2s_lcd_setup_v2(config_t& _cfg) // The big one that gets everything setup.
  {
      auto dev = getDev();
      volatile int iomux_signal_base;
      volatile int iomux_clock;
      int irq_source;

      periph_module_reset(PERIPH_I2S0_MODULE);
      periph_module_enable(PERIPH_I2S0_MODULE);

      iomux_clock = I2S0O_WS_OUT_IDX;
      irq_source = ETS_I2S0_INTR_SOURCE;

      if ( _cfg.parallel_width == 24)
      {
        ESP_LOGI(TAG, "Configuring signal base for 24bit");
        iomux_signal_base = I2S0O_DATA_OUT0_IDX;

      } else {

        ESP_LOGI(TAG, "Configuring signal base for 16bit");
        iomux_signal_base = I2S0O_DATA_OUT8_IDX;
      }

      // Setup GPIOs
      int bus_width = _cfg.parallel_width;

      // Clock output GPIO setup
      _gpio_pin_init(_cfg.pin_wr); // clock

      // Data output GPIO setup
      int8_t* pins = _cfg.pin_data;  

      for(int i = 0; i < bus_width; i++)  {
      _gpio_pin_init(pins[i]);
      }

      // Route clock signal to clock pin (can route to two pins if we want)
      gpio_matrix_out(_cfg.pin_wr, iomux_clock, _cfg.invert_pclk, 0); // inverst clock if required

      // Route data pins
      for (size_t i = 0; i < bus_width; i++) {
        if (pins[i] >= 0) {
          gpio_matrix_out(pins[i], iomux_signal_base + i, false, false);
        }
      }

      ////////////////////////////// Clock configuration //////////////////////////////
      // Code borrowed from: https://github.com/espressif/esp-iot-solution/blob/master/components/bus/i2s_lcd_esp32s2_driver.c
      unsigned int _div_num = (unsigned int) (160000000L / _cfg.bus_freq / i2s_parallel_get_memory_width(ESP32_I2S_DEVICE, bus_width));       

      ESP_LOGI(TAG, "Clock divider is: %d", _div_num);

      // Configure the clock
      dev->clkm_conf.val = 0;
      //dev->clkm_conf.clkm_div_num = 2; // 160MHz / 2 = 80MHz

      dev->clkm_conf.clkm_div_num = _div_num; 
      dev->clkm_conf.clkm_div_b = 0;
      dev->clkm_conf.clkm_div_a = 63;
      dev->clkm_conf.clk_sel = 2;
      dev->clkm_conf.clk_en = 1;

      // Configure sampling rate
      //dev->sample_rate_conf.tx_bck_div_num = 40000000 / 2000000; // Fws = Fbck / 2
      dev->sample_rate_conf.tx_bck_div_num = 4;
      dev->sample_rate_conf.tx_bits_mod = _cfg.parallel_width;

      dev->timing.val = 0;

      dev->int_ena.val = 0;
      dev->int_clr.val = ~0;

      dev->conf2.val = 0;
      dev->conf2.lcd_en = 1;

      // Configuration data format
      dev->conf.val = 0;
      dev->conf.tx_right_first = 1;
      dev->conf.tx_msb_right = 1;
      dev->conf.tx_dma_equal = 1;

      dev->conf1.tx_pcm_bypass = 1;
      dev->conf1.tx_stop_en = 1;

      dev->fifo_conf.val = 0;
      dev->fifo_conf.tx_data_num = 32;
      dev->fifo_conf.dscr_en = 1;    

      // Not requried for S2?
      dev->fifo_conf.tx_fifo_mod_force_en = 1;       
      dev->fifo_conf.tx_fifo_mod = 1;      
      dev->conf_chan.tx_chan_mod = 0;//remove


      dev->lc_conf.ext_mem_bk_size = 0;
      dev->fifo_conf.tx_24msb_en = 0;


      //dev->int_ena.out_eof = 1;
    
      return ESP_OK;
    }


  // VERSION 2!
  esp_err_t dma_allocate_v2(config_t& _cfg)
  {
      // The framebuffer / payload size needs to match the size of the parallel_wifth!
      size_t fb_size  = 2000; // 16 bit / 2 byte mode

      fb_size *= (_cfg.parallel_width/8);

      // fb caps
      int dma_align = ll_cam_get_dma_align();

      /* Allocate memory for frame buffer */
      size_t alloc_size = fb_size * sizeof(DMA_DATA_TYPE) + dma_align;

      //uint32_t _caps = MALLOC_CAP_SPIRAM;
      uint32_t _caps = MALLOC_CAP_INTERNAL  | MALLOC_CAP_DMA;

  #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
          // In IDF v4.2 and earlier, memory returned by heap_caps_aligned_alloc must be freed using heap_caps_aligned_free.
          // And heap_caps_aligned_free is deprecated on v4.3.
          global_buffer = (DMA_DATA_TYPE *)heap_caps_aligned_alloc(dma_align, alloc_size, _caps);
  #else
          global_buffer = (DMA_DATA_TYPE *)heap_caps_malloc(alloc_size, _caps);
  #endif

      if (global_buffer == NULL)  {
              ESP_LOGE(TAG, "xxxx Frame buffer malloc failed.");
      } 

      memset(global_buffer, 0, alloc_size); // zero it.

      ESP_LOGI(TAG, "Global Buffer Addr: 0x%08X", (uintptr_t) global_buffer);

      // fb offset
      size_t fb_offset = dma_align - ((uintptr_t)global_buffer & (dma_align - 1));
      //global_buffer += fb_offset; // increment pointer
      ESP_LOGI(TAG, "Global Buffer: Offset: %u, Addr: 0x%08X", fb_offset, (uintptr_t) global_buffer);

      // dma ll desc
      int dma_node_cnt = ll_desc_get_required_num(fb_size); // Number of DMA nodes  8000 / 4092

      global_buffer += fb_offset; // increment pointer

      ESP_LOGI(TAG, "Global Buffer Addr (aligned): 0x%08X", (uintptr_t) global_buffer);


      dma_ll =  allocate_dma_descriptors_gb(dma_node_cnt, fb_size, global_buffer); 

      // Pattern Length.          
    
    /*
      for (int i = 0; i < fb_size; i++) {
        // 17 bits
        global_buffer[i] = (i%21) ? 0b0000000000000000:0xffff; //0b1000000100000010;
      }
      */

      global_buffer[75] = 0b1; // byte 1
      global_buffer[76] = 0b01000000; // byte 2
      global_buffer[77] = 0xff; // byte 3
      
/*
      int val = 0;
      for (int i = 0; i < (fb_size-3); i +=3) {
        // 17 bits
        val = !val;
        global_buffer[i]   = (val) ? 0:1; //0b1000000100000010;
        //global_buffer[i+1] = (val) ? 0:0; //0b1000000100000010;
        //global_buffer[i+2] = (val) ? 0:0; //0b1000000100000010;                        
      }     
  */    
      /* 
      uint8_t pattern[] = {0b00, 0b11, 0b11, 0b11};
      int pattern_length   = sizeof(pattern);
      ESP_LOGI(TAG, "Pattern length is %d bytes", pattern_length);

      // Fill memory with repeating pattern
        size_t filled_bytes = 0;
        while (filled_bytes < fb_size) {
            size_t remaining_bytes = fb_size - filled_bytes;
            size_t bytes_to_copy = remaining_bytes < pattern_length ? remaining_bytes : pattern_length;
           // memcpy( (global_buffer + filled_bytes), pattern, bytes_to_copy);
            filled_bytes += bytes_to_copy;
        }     
      */
      return ESP_OK;

  } // dma_allocate



    esp_err_t dma_start_v2()
    {
      auto dev = getDev();

      while (!dev->state.tx_idle);
      dev->conf.tx_start = 0;
      dev->conf.tx_reset = 1;
      dev->conf.tx_reset = 0;
      dev->conf.tx_fifo_reset = 1;
      dev->conf.tx_fifo_reset = 0;
      dev->out_link.addr = ((uint32_t)&dma_ll[0]) & 0xfffff; // ((uint32_t)&frames[0].dma[0]) & 0xfffff; // always frame 0
      dev->out_link.start = 1;
      ets_delay_us(1);
      dev->conf.tx_start = 1;

    
      // Configure DMA burst mode
      //dev->lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN;

      return ESP_OK;

    } // end 
    

  /*
    // from ll_cam.c in the esp32-camera example
    
    I2S0.rx_eof_num = cam->dma_half_buffer_size; // Ping pong operation
      if (!cam->psram_mode) {
          I2S0.in_link.addr = ((uint32_t)&cam->dma[0]) & 0xfffff;
      } else {
          I2S0.in_link.addr = ((uint32_t)&cam->frames[frame_pos].dma[0]) & 0xfffff;
      }
  */


  /*

  void IRAM_ATTR spicommon_dma_desc_setup_link(spi_dma_desc_t *dmadesc, const void *data, int len, bool is_rx)
  {
      dmadesc = ADDR_DMA_2_CPU(dmadesc);
      int n = 0;
      while (len) {
          int dmachunklen = len;
          if (dmachunklen > DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED) {
              dmachunklen = DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED;
          }
          if (is_rx) {
              //Receive needs DMA length rounded to next 32-bit boundary
              dmadesc[n].dw0.size = (dmachunklen + 3) & (~3);
          } else {
              dmadesc[n].dw0.size = dmachunklen;
              dmadesc[n].dw0.length = dmachunklen;
          }
          dmadesc[n].buffer = (uint8_t *)data;
          dmadesc[n].dw0.suc_eof = 0;
          dmadesc[n].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
          dmadesc[n].next = ADDR_CPU_2_DMA(&dmadesc[n + 1]);
          len -= dmachunklen;
          data += dmachunklen;
          n++;
      }
      dmadesc[n - 1].dw0.suc_eof = 1; //Mark last DMA desc as end of stream.
      dmadesc[n - 1].next = NULL;
  }


  */