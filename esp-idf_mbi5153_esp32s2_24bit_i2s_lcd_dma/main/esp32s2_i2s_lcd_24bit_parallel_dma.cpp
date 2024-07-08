#include <sdkconfig.h>

#if !defined (CONFIG_IDF_TARGET_ESP32S2)
  #pragma error "Designed only for ESP32-S2"
#endif

#include "esp32s2_i2s_lcd_24bit_parallel_dma.hpp"

#include "mbi_gclk_addr_data_full_rate.h"
  

#include <driver/gpio.h>
#if (ESP_IDF_VERSION_MAJOR == 5)
#include <esp_private/periph_ctrl.h>
#else
// Break the compile 
DO NOT COMPILE
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
#include <rom/ets_sys.h> // ets delay

// New dma helper tools
#include "esp_dma_utils.h"
#include "rom/cache.h"

#include "app_constants.h"

// #define USE_PSRAM 1  // You really don't want to do this on the S2 using EDBA, it's shit slow. Max 1Mhz output!


static const char *TAG = "i2s_24bit_lcd";

/*************** Buffer Lengths ****************/

// The length in parallel clocks. Actual memory use will be 3 x this.
#define BUFF_BITLEN_VSYNC             500    // Half rate data bit length + some clocks for vsync

// Double the time as we don't have enough to send colour data in the time it takes to iterate through all the rows and stimulate LEDs + some clocks for vsync
#define BUFF_BITLEN_GCLK_CDATA       ((20520*2)+1000)
#define BUFF_BITLEN_CONFIG           (200)   // Config stuff

/******************* Externs  ******************/
DMA_DATA_TYPE *global_buffer_gclk_cdata   = NULL; // data for gclk + new payload of MBI serial colour data + vsync
DMA_DATA_TYPE *global_buffer_vsync        = NULL;
DMA_DATA_TYPE *global_buffer_configuration      = NULL; // for sending any other data

static int gclk_cdata_last_dma_ll_desc_pos = 0;
lldesc_t *dma_ll_gclk_cdata = NULL;
lldesc_t *dma_ll_vsync = NULL;
lldesc_t *dma_ll_configuration = NULL;

/******************* ******  *******************/
  static volatile bool dma_buffer_sent = false;
  static volatile int  interrupt_count = 0;

/******************* ******  *******************/
    #define ESP32_I2S_DEVICE I2S_NUM_0	

    esp_err_t mbi_start_output_loop();

    // Static
    i2s_dev_t* getDev()
    {
        return &I2S0;
    }


/******************* ******  *******************/
    static void IRAM_ATTR i2s_dma_isr(void* arg) {

      interrupt_count = interrupt_count + 1;

      // set the greyscale / gclk to loop back on itself again please
      dma_ll_gclk_cdata[gclk_cdata_last_dma_ll_desc_pos].empty = (uintptr_t)&dma_ll_gclk_cdata[0]; // loop greyscale back on itself in the interrupt.   

      // Clear flag so we can get retriggered
      SET_PERI_REG_BITS(I2S_INT_CLR_REG(I2S_NUM_0), I2S_OUT_EOF_INT_CLR_V,  1, I2S_OUT_EOF_INT_CLR_S);                      
      SET_PERI_REG_BITS(I2S_INT_CLR_REG(I2S_NUM_0), I2S_OUT_DONE_INT_CLR_V, 1, I2S_OUT_DONE_INT_CLR_S);                
      
      // at this point, the previously active buffer is free, go ahead and write to it
      dma_buffer_sent = true;
    }

    int get_interrupt_count(){
      return interrupt_count;
    }


    void _mbl_set_latch_for_config_buff(int latch_length, int &start_pos, bool stay_low = false) {

        for (int i = latch_length; i > 0; i--) {
          global_buffer_configuration[start_pos++].lat = (stay_low) ? 0:1;
#ifdef USE_PSRAM          
          Cache_WriteBack_Addr((uint32_t) &global_buffer_configuration[start_pos-1], sizeof(DMA_DATA_TYPE)); // Ensure written to PSRAM, needs to be a &reference[x].byteX !         
#endif          
        }
    }

    /* Need to set config for all MBI input on the panel at once as they all share the same latch pin */
    void _mbi_set_config_dma(int &dma_output_pos, uint16_t config_reg, bool latch, bool reg2) {

      ESP_LOGD(TAG, "_mbi_set_config_dma()");

      // Number of DCLK Rising Edge when LE is asserted
      // Write Configuration 1 = 4
      // Write Configuration 2 = 8
      int latch_trigger_point = (reg2) ? 8 : 4;

      for (int bit = 15; bit >= 0; bit--) {
        int bitval = ((config_reg >> bit) & 1);

          if (bitval) {
            // All PWM chips across all colour channels get the same config for now.
            global_buffer_configuration[dma_output_pos].b1 = global_buffer_configuration[dma_output_pos].b2 = global_buffer_configuration[dma_output_pos].b3 = global_buffer_configuration[dma_output_pos].b4 = 1;
            global_buffer_configuration[dma_output_pos].r1 = global_buffer_configuration[dma_output_pos].r2 = global_buffer_configuration[dma_output_pos].r3 = global_buffer_configuration[dma_output_pos].r4 = 1;
            global_buffer_configuration[dma_output_pos].g1 = global_buffer_configuration[dma_output_pos].g2 = global_buffer_configuration[dma_output_pos].g3 = global_buffer_configuration[dma_output_pos].g4 = 1;

          }

          if ((bit < latch_trigger_point) && (latch == true)) {  // for reg1, data latch on the last 4 bits when latch is true (on last send)
            global_buffer_configuration[dma_output_pos].lat = 1;
          }
#ifdef USE_PSRAM          
          // Write to PSRAM
          Cache_WriteBack_Addr((uint32_t) &global_buffer_configuration[dma_output_pos], sizeof(DMA_DATA_TYPE)); // Ensure written to PSRAM, needs to be a &reference[x].byteX !         
#endif
          dma_output_pos++; // increment

      }  // iterate through bits
    }  // mbi_send_config_dma

    void _mbi_load_config_reg1(int &buff_start_pos) {

      int ghost_elimination   = 0b11;
      int line_num            = 20 - 1;
      int gray_scale          = 0; // 14 bit
      int gclk_multiplier     = 0; // multiplier off
      int current             = 15;  // 0-63 is the range. 15 is good.

      // Documentation says set bits E and F of Config1 Reg to 1
      uint16_t config_reg1_val = 0; 
      config_reg1_val = (config_reg1_val | (ghost_elimination << 14) | (line_num << 8) | (gray_scale << 7) | (gclk_multiplier << 6) | (current));

      for (int i = 0; i < PANEL_MBI_CHAIN_LEN; i++) {
        _mbi_set_config_dma(buff_start_pos, config_reg1_val, (i == (PANEL_MBI_CHAIN_LEN - 1)), false);  // on last mbi chip, latch
      }

    }  // mbi_send_config_reg1_dma

    void _mbi_load_config_reg2(int &buff_start_pos) {

      // F E D C B A 9 8 7 6 5 4 3 2 1 0
      // uint16_t config_reg2_val = 0b0001000000010000; // default value apparently      
      uint16_t config_reg2_val = 0b1001000000011110;  // removes ghosting except for red

      for (int i = 0; i < PANEL_MBI_CHAIN_LEN; i++) {
        _mbi_set_config_dma(buff_start_pos, config_reg2_val, (i == (PANEL_MBI_CHAIN_LEN - 1)), true);  // on last mbi chip, latch
      }
    }  // mbi_send_config_reg2_dma

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
    inline int i2s_parallel_get_memory_width(int width) {
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


  /* @brief Split the payload up and link to from x DMA descriptors. Number of descriptors needs to have been calculated correctly by ll_desc_get_required_num
    *
    * @param uint32_t count   : Number of DMA descriptors required
    * @param size_t   payload : Total size in (bytes) of payload to be linked to.
    * @param void *buffer     : Pointer to the first address of contigurious chunk of payload data.
    * @param bool loop_back   : Loop output endlessly by linking last DMA descriptor to first.
    * 
    * @return Pointer to lldesc_t malloc of size: count * sizeof(lldesc_t)
    */
    static lldesc_t * allocate_dma_descriptors_gb(uint32_t count, size_t payload_size, void *buffer, bool loop_back = true)
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

              if (loop_back) {
                dma[n].empty = (uintptr_t)&dma[0];
                ESP_LOGD("allocate_dma_descriptors()", "Linking lldesc_t  pos %d back to pos 0", n);      
              }
              else
              {
                dma[n].empty = 0; // null
                ESP_LOGD("allocate_dma_descriptors()", "Terminating DMA descriptor linked list (EOF) at pos %d.", n);      
              }

            } else {
              dma[n].empty = (uintptr_t)&dma[(n  + 1) % count];
              ESP_LOGD("allocate_dma_descriptors()", "Linking lldesc_t  pos %d to pos %d at memory location %08x.", n, (n+1), (uintptr_t)&dma[(n + 1) % count]);      
            }
              ESP_LOGD("allocate_dma_descriptors()", "Chunk len %d.", dmachunklen);          


          n++;


        }

        return dma;
    }

    static int ll_desc_get_required_num(uint32_t bytes_len)
    {
        int ll_desc_required = (bytes_len + DMA_MAX - 1) / DMA_MAX;
        ESP_LOGD("ll_desc_get_required_num()", "Gunna need %d dma lldesc's", ll_desc_required);      

        return ll_desc_required;
    }

    esp_err_t i2s_lcd_mode_setup() // The big one that gets everything setup.
    {
      esp_err_t ret;

        // START: CONFIGURATION
        config_t _cfg;


        _cfg.bus_freq = 6*1000*1000; // Do not go above 6Mhz
        _cfg.parallel_width = 24;

        _cfg.pin_wr      = MBI_DCLK_PIN; // lcd clock
        _cfg.invert_pclk = false;

        // Ensure this order matches that of the struct from lsb to msb.

        _cfg.pin_d0  = MBI_GCLK_PIN; 
        _cfg.pin_d1  = ADDR_A_PIN; 
        _cfg.pin_d2  = ADDR_B_PIN; 
        _cfg.pin_d3  = ADDR_C_PIN; 
        _cfg.pin_d4  = ADDR_D_PIN; 
        _cfg.pin_d5  = ADDR_E_PIN; 
        _cfg.pin_d6  = -1;
        _cfg.pin_d7  = MBI_LAT_PIN;
        _cfg.pin_d8  = MBI_R1_PIN;  // start of second byte
        _cfg.pin_d9  = MBI_G1_PIN;
        _cfg.pin_d10 = MBI_B1_PIN;
        _cfg.pin_d11 = MBI_R2_PIN;
        _cfg.pin_d12 = MBI_G2_PIN;  
        _cfg.pin_d13 = MBI_B2_PIN; 
        _cfg.pin_d14 = -1;  
        _cfg.pin_d15 = -1; // end of byte 2
        _cfg.pin_d16 = MBI_R3_PIN; // start of third byte
        _cfg.pin_d17 = MBI_G3_PIN;
        _cfg.pin_d18 = MBI_B3_PIN;
        _cfg.pin_d19 = MBI_R4_PIN;
        _cfg.pin_d20 = MBI_G4_PIN;      
        _cfg.pin_d21 = MBI_B4_PIN;        
        _cfg.pin_d22 = -1;          
        _cfg.pin_d23 = -1;  // end of third byte

        // END CONFIGURATION
              
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
        //unsigned int _div_num = (unsigned int) (160000000L / _cfg.bus_freq / i2s_parallel_get_memory_width(ESP32_I2S_DEVICE, bus_width));       

        // Configure the clock
        dev->clkm_conf.val = 0;
        
#ifdef USE_PSRAM
        dev->clkm_conf.clkm_div_num = (160/5); // Effectively 1.2Mhz
#else
        dev->clkm_conf.clkm_div_num = 7; // 160/7/3bytes = 5mhz
#endif        
                
        ESP_LOGD(TAG, "I2S clock divider (clkm_div_num) is: %d", (dev->clkm_conf.clkm_div_num));

#ifndef USE_PSRAM        
        int effective_frequency = (160 / dev->clkm_conf.clkm_div_num) / i2s_parallel_get_memory_width(24);
        ESP_LOGD(TAG, "I2S effective output frequency is: %d Mhz", effective_frequency);        
#endif 

        dev->clkm_conf.clkm_div_b = 0;
        dev->clkm_conf.clkm_div_a = 63;
        dev->clkm_conf.clk_sel = 2;
        dev->clkm_conf.clk_en = 1;

        // Configure sampling rate
        //dev->sample_rate_conf.tx_bck_div_num = 40000000 / 2000000; // Fws = Fbck / 2

        /* ESP32-S2 TRM IS CLEAR: Note that I2S_TX_BCK_DIV_NUM[5:0] must not be configured as 1.*/
        dev->sample_rate_conf.tx_bck_div_num = 2; 

        dev->sample_rate_conf.tx_bits_mod = _cfg.parallel_width;

        dev->timing.val = 0;

        dev->int_ena.val = 0;
        dev->int_clr.val = ~0;

        dev->conf2.val = 0;
        dev->conf2.lcd_en = 1;

        // Configuration data format
        dev->conf.val = 0;
      // dev->conf.tx_right_first = 1; // doesn't change anything if 0
      // dev->conf.tx_msb_right = 1;   // doesn't change anything if 0
        dev->conf.tx_dma_equal = 1;

        dev->conf1.tx_pcm_bypass = 1;
        dev->conf1.tx_stop_en = 1;

        dev->fifo_conf.val = 0;
        dev->fifo_conf.tx_data_num = 32;
        dev->fifo_conf.dscr_en = 1;    

        // Not requried for S2?
        dev->fifo_conf.tx_fifo_mod_force_en = 1;       
        dev->fifo_conf.tx_fifo_mod = 1;   // frequenc
        dev->conf_chan.tx_chan_mod = 0;//remove


        dev->lc_conf.ext_mem_bk_size = 0;
        dev->fifo_conf.tx_24msb_en = 0;

        //dev->int_ena.out_done = 1;
        dev->int_ena.out_eof  = 1;

        // Setup I2S Interrupt
        //SET_PERI_REG_BITS(I2S_INT_ENA_REG(I2S_NUM_0), I2S_OUT_EOF_INT_ENA_V, 1, I2S_OUT_EOF_INT_ENA_S);

        // Allocate a level 1 intterupt: lowest priority, as ISR isn't urgent and may take a long time to complete
        ret = esp_intr_alloc(irq_source, (int)(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1), i2s_dma_isr, NULL, NULL);
      
        return ret;
      }

    esp_err_t i2s_dma_buff_allocate()
    {
 
        /**********************************************************************************/
        // Allocate the buffer for new greyscale data, at the same time as sending out gclks,
        // and finishing off with a vsync.
        {
          size_t alloc_size_bytes  = BUFF_BITLEN_GCLK_CDATA * sizeof(DMA_DATA_TYPE); // Up to 44,000  pulses of 24 bits in parallel.
          size_t actual_size = 0;

#ifdef USE_PSRAM
          ESP_LOGI(TAG, "Allocating PSRAM DMA memory for global_buffer_gclk_cdata.");  
          esp_err_t err = esp_dma_malloc(alloc_size_bytes, ESP_DMA_MALLOC_FLAG_PSRAM, (void **) &global_buffer_gclk_cdata, &actual_size);
          assert(err == ESP_OK);
#else
          ESP_LOGI(TAG, "Allocating internal SRAM DMA memory for global_buffer_gclk_cdata.");  
          global_buffer_gclk_cdata = (DMA_DATA_TYPE *)heap_caps_malloc(alloc_size_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
          assert(global_buffer_gclk_cdata != nullptr);
#endif

          size_t alignment_offset = actual_size - alloc_size_bytes;      

          ESP_LOGI(TAG, "Actual size is: %d bytes", actual_size);  
          ESP_LOGI(TAG, "Alignment offset is: %d ", alignment_offset);        

          if (global_buffer_gclk_cdata == NULL)  {
                  ESP_LOGE(TAG, "global_buffer_gclk_cdata malloc failed.");
          } 

          // Zero out
          memset(global_buffer_gclk_cdata, 0, alloc_size_bytes); // zero it. 

#ifdef USE_PSRAM                         
          Cache_WriteBack_Addr((uint32_t) global_buffer_gclk_cdata, alloc_size_bytes);   
#endif

          // dma ll desc
          int dma_node_cnt = ll_desc_get_required_num(alloc_size_bytes); 
          dma_ll_gclk_cdata =  allocate_dma_descriptors_gb(dma_node_cnt, alloc_size_bytes, global_buffer_gclk_cdata, true);

          // Last linked list item for this buffer points to global_buffer_gclk_only!! Only one way 
          gclk_cdata_last_dma_ll_desc_pos = dma_node_cnt-1;

          // Populate with data
          int start_pos = BUFF_BITLEN_GCLK_CDATA-500;
          int latch_length = 3; 
          for (int i = latch_length; i > 0; i--) {        
            global_buffer_gclk_cdata[start_pos++].lat = 1;                      
          }



        }

        //
        // Allocate Vsync Buffer
        //
        {
          size_t alloc_size_bytes  = BUFF_BITLEN_VSYNC * sizeof(DMA_DATA_TYPE); // Up to 44,000  pulses of 24 bits in parallel.

          ESP_LOGI(TAG, "Allocating internal SRAM DMA memory for global_buffer_gclk_cdata.");  
          global_buffer_vsync = (DMA_DATA_TYPE *)heap_caps_malloc(alloc_size_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
          assert(global_buffer_vsync != nullptr);

          // Zero out
          memset(global_buffer_vsync, 0, alloc_size_bytes); // zero it. 

          // dma ll desc
          int dma_node_cnt = ll_desc_get_required_num(alloc_size_bytes); // it will always be 1 or zero!
          dma_ll_vsync =  allocate_dma_descriptors_gb(dma_node_cnt, alloc_size_bytes, global_buffer_vsync);
          dma_ll_vsync[0].eof = 1; // trigger eof only time this should happen

/*
          // Populate with data
          int start_pos = BUFF_BITLEN_VSYNC/2;
          int latch_length = 3; 
          for (int i = latch_length; i > 0; i--) {        
            global_buffer_vsync[start_pos++].lat = 1;                      
          }

*/

        }

        // Allocate the buffer for command and other crap
        {
          size_t alloc_size_bytes  = BUFF_BITLEN_CONFIG * sizeof(DMA_DATA_TYPE); 
          size_t actual_size = 0;

#ifdef USE_PSRAM
          ESP_LOGI(TAG, "Allocating PSRAM DMA memory for global_buffer_configuration.");  
          esp_err_t err = esp_dma_malloc(alloc_size_bytes, ESP_DMA_MALLOC_FLAG_PSRAM, (void **) &global_buffer_configuration, &actual_size);
          assert(err == ESP_OK);
#else
          ESP_LOGI(TAG, "Allocating internal SRAM DMA memory for global_buffer_configuration.");  
          global_buffer_configuration = (DMA_DATA_TYPE *)heap_caps_malloc(alloc_size_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
          assert(global_buffer_configuration != nullptr);
#endif
          size_t alignment_offset = actual_size - alloc_size_bytes;      

          ESP_LOGI(TAG, "Actual size is: %d bytes", actual_size);  
          ESP_LOGI(TAG, "Alignment offset is: %d ", alignment_offset);        

          if (global_buffer_configuration == NULL)  {
                  ESP_LOGE(TAG, "global_buffer_configuration malloc failed.");
          } 

          // Zero out
          memset(global_buffer_configuration, 0, alloc_size_bytes); // zero it.    

#ifdef USE_PSRAM                      
          Cache_WriteBack_Addr((uint32_t) global_buffer_configuration, alloc_size_bytes);   
#endif          

          // dma ll desc
          int dma_node_cnt = ll_desc_get_required_num(alloc_size_bytes); // Number of DMA nodes  xxxxx / 4092
          dma_ll_configuration =  allocate_dma_descriptors_gb(dma_node_cnt, alloc_size_bytes, global_buffer_configuration, false); //,false);     

        }       

        return ESP_OK;

    } // dma_allocate

    esp_err_t mbi_load_inital_dma_data() {
        { // block 1

              // Part 1 - Load the GCLK data
              // NOTE: As we use a bit in byte0 for the LATCH as well, we need to ensure this is written after this is loaded.
              int bitlen = sizeof(dma_gclk_addr_full_clk_rate);
              for (int i = 0; i < bitlen; i++) {
                    // Load it twice
                    global_buffer_gclk_cdata[i].byte0         = dma_gclk_addr_full_clk_rate[i];            
                    global_buffer_gclk_cdata[bitlen+i].byte0 =  dma_gclk_addr_full_clk_rate[i];            

#ifdef USE_PSRAM                              
                    Cache_WriteBack_Addr((uint32_t) &global_buffer_gclk_cdata[i], sizeof(DMA_DATA_TYPE)); // Ensure written to PSRAM, needs to be a &reference[x].byteX !   
                    Cache_WriteBack_Addr((uint32_t) &global_buffer_gclk_cdata[bitlen+i], sizeof(DMA_DATA_TYPE)); // Ensure written to PSRAM, needs to be a &reference[x].byteX !                     
#endif                    
              }

              // Part 2, the greyscale data latches in the right spot. Assume greyscale data will start from postion 0 in this buffer.
              {
              int counter = 0;
              for (int row = 0; row < PANEL_SCAN_LINES; row++) { // rows
                for (int chan = 0; chan < PANEL_MBI_LED_CHANS; chan++) { // channels per chip
                  for (int ic = 0; ic < PANEL_MBI_CHAIN_LEN; ic++) {  // number of chained ICs

                   global_buffer_gclk_cdata[counter].lat = 0;

                    // data latch on the last bit, when sending the last byte set latch=1
                    int latch = 0;
                    if (ic == 4) {
                      latch = 1;
                    }  // latch on last channel / ic

                    int bit_offset = 16;
                    while (bit_offset > 0)  // shift out MSB first per the documentation.
                    {
                      bit_offset--;  // start from 15

                      if (latch == 1 && bit_offset == 0) {
                        global_buffer_gclk_cdata[counter].lat = 1;
              #ifdef USE_PSRAM                                  
                        Cache_WriteBack_Addr((uint32_t) &global_buffer_gclk_cdata[counter], sizeof(DMA_DATA_TYPE)); // Ensure written to PSRAM, needs to be a &reference[x].byteX !                               
              #endif                        
                      }
                      counter++;
                    }
                  }
                }
              } // end latch config for greyscale data      
              }      

        } //end block 1

        // Load configuration / Start-up Data
        {
              int start_pos = 10;
              ESP_LOGI(TAG, "0. Start Pos is: %d.", start_pos);              

              // Send Reset - 10 clocks of dclock rising edge
              _mbl_set_latch_for_config_buff(10, start_pos); 
              ESP_LOGI(TAG, "1. Start Pos is: %d.", start_pos);              

              // Stay low for a bit
              _mbl_set_latch_for_config_buff(2, start_pos, true); // stay low
              ESP_LOGI(TAG, "2. Start Pos is: %d.", start_pos);              

              // Send preactive - 14 clocks of dclock rising edge
              _mbl_set_latch_for_config_buff(14, start_pos); // stay low
              ESP_LOGI(TAG, "3. Start Pos is: %d.", start_pos);                          

              // Stay low for a bit
              _mbl_set_latch_for_config_buff(6, start_pos, true); // stay low
              ESP_LOGI(TAG, "4. Start Pos is: %d.", start_pos);         

              start_pos += 16; // add a bit of space                 

              _mbi_load_config_reg1(start_pos);
              ESP_LOGI(TAG, "5. Start Pos is: %d.", start_pos);      

              // Stay low for a bit
              _mbl_set_latch_for_config_buff(2, start_pos, true); // stay low
              ESP_LOGI(TAG, "5a. Start Pos is: %d.", start_pos);                  

              _mbl_set_latch_for_config_buff(10, start_pos); // reset
              ESP_LOGI(TAG, "6. Start Pos is: %d.", start_pos);                                    

              // 
              //     // Send preactive - 14 clocks of dclock rising edge
              //     _mbl_set_latch_for_config_buff(14, start_pos); // stay low
              //     ESP_LOGI(TAG, "6. Start Pos is: %d.", start_pos);                          

              //     // Stay low for a bit
              //     _mbl_set_latch_for_config_buff(2, start_pos, true); // stay low
              //     ESP_LOGI(TAG, "7. Start Pos is: %d.", start_pos);                          

              //     _mbi_load_config_reg1(start_pos);
              //     ESP_LOGI(TAG, "8. Start Pos is: %d.", start_pos);                          
              // 
        } // start-up


      // Load just GCLK data
      return ESP_OK;
    }    

    esp_err_t mbi_send_config()
    {
      auto dev = getDev();

      // Configure DMA burst mode
      dev->lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN;
     

      while (!dev->state.tx_idle);
      dev->conf.tx_start = 0;
      dev->conf.tx_reset = 1;
      dev->conf.tx_reset = 0;
      dev->conf.tx_fifo_reset = 1;
      dev->conf.tx_fifo_reset = 0;

      // ensure to link to the dma_ll var!!
      dev->out_link.addr = ((uint32_t)&dma_ll_configuration[0]) & 0xfffff;
      dev->out_link.start = 1;
      ets_delay_us(1);
      dev->conf.tx_start = 1;

      return ESP_OK;

    } // end 


  esp_err_t mbi_send_vsync()
    {
      auto dev = getDev();

      // Configure DMA burst mode
      dev->lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN;
     

      while (!dev->state.tx_idle);
      dev->conf.tx_start = 0;
      dev->conf.tx_reset = 1;
      dev->conf.tx_reset = 0;
      dev->conf.tx_fifo_reset = 1;
      dev->conf.tx_fifo_reset = 0;

      // ensure to link to the dma_ll var!!
      dev->out_link.addr = ((uint32_t)&dma_ll_vsync[0]) & 0xfffff;
      dev->out_link.start = 1;
      ets_delay_us(1);
      dev->conf.tx_start = 1;

      return ESP_OK;

    } // end     

    esp_err_t mbi_start_output_loop()
    {
      auto dev = getDev();

      // Configure DMA burst mode
      dev->lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN;      

      while (!dev->state.tx_idle);
      dev->conf.tx_start = 0;
      dev->conf.tx_reset = 1;
      dev->conf.tx_reset = 0;
      dev->conf.tx_fifo_reset = 1;
      dev->conf.tx_fifo_reset = 0;

      // ensure to link to the dma_ll var!!
      dev->out_link.addr = ((uint32_t)&dma_ll_gclk_cdata[0]) & 0xfffff; // ((uint32_t)&frames[0].dma[0]) & 0xfffff; // always frame 0
      dev->out_link.start = 1;
      ets_delay_us(1);
      dev->conf.tx_start = 1;

      return ESP_OK;

    } // end 

    void mbi_clear()
    {
        for (int i = 0; i< BUFF_BITLEN_GCLK_CDATA; i++) {

          global_buffer_gclk_cdata[i].byte1 = 0; 
          global_buffer_gclk_cdata[i].byte2 = 0; 

          // if we use PSRAM need to add that cache writeback bullshite here.
          // PSRAM on S2 sucks ballz so not going to bother.
        }

    }

    void mbi_set_pixel(uint8_t x, uint8_t y, uint8_t r_data, uint8_t g_data, uint8_t b_data) {

      if (x >= PANEL_PHY_RES_X || y >= PANEL_PHY_RES_Y) {
        return;
      }

      x += 2;  // offset for missing pixels on the left

      // Row offset + channel offset + individual IC LED offset
      // Calculate data array start position
      int y_normalised  = y % PANEL_SCAN_LINES;  // Only have 20 rows of data...
      int bit_start_pos = (1280 * y_normalised) + ((x % 16) * 80) + ((x / 16) * 16);  

      int rgb_channel = (y / PANEL_SCAN_LINES) + 1;   // three is an important bit

      /*
        MBI5153 provides a selectable 14-bit or 13-bit gray scale by setting the configuration register1 bit [7]. The default 
        value is set to ’0’ for 14-bit color depth. In 14-bit gray scale mode, users should still send 16-bit data with 2-bit ‘0’ in 
        LSB bits. For example, {14’h1234, 2’h0}. 
      */    

      // RGB colour data provided is only 8bits, so we'll fill it from bit 16 down to bit 8
      // 14-bit resolution = 16,384
      //  8-bit resolutoin = 255     
      int subpixel_colour_bit = 8;
      int offset = 0;

      //ESP_LOGD(TAG, "RGB Channel is is %d", rgb_channel);

      switch (rgb_channel)
      {
        case 1:
        {
            while (subpixel_colour_bit > 0) { // shift out MSB first per the documentation.
              subpixel_colour_bit--;  // start from 7
              uint8_t mask = 1 << subpixel_colour_bit;

              global_buffer_gclk_cdata[bit_start_pos+offset].r1 = (mask & r_data ) ? 1:0; // (r_data & mask);      
              global_buffer_gclk_cdata[bit_start_pos+offset].g1 = (mask & g_data ) ? 1:0; //(g_data & mask);      
              global_buffer_gclk_cdata[bit_start_pos+offset].b1 = (mask & b_data ) ? 1:0;// (g_data & mask);      
              offset++;
            }
        } // rgb1
        break;

        case 2:
        {
            while (subpixel_colour_bit > 0) { // shift out MSB first per the documentation.
              subpixel_colour_bit--;  // start from 7
              uint8_t mask = 1 << subpixel_colour_bit;

              global_buffer_gclk_cdata[bit_start_pos+offset].r2 = (mask & r_data ) ? 1:0   ;   
              global_buffer_gclk_cdata[bit_start_pos+offset].g2 = (mask & g_data ) ? 1:0   ;       
              global_buffer_gclk_cdata[bit_start_pos+offset].b2 = (mask & b_data ) ? 1:0   ;      
              offset++;
            }
        } // rgb2
        break;

        case 3:
        {
            while (subpixel_colour_bit > 0) { // shift out MSB first per the documentation.
              subpixel_colour_bit--;  // start from 7
              uint8_t mask = 1 << subpixel_colour_bit;

              global_buffer_gclk_cdata[bit_start_pos+offset].r3 = (mask & r_data ) ? 1:0   ;        
              global_buffer_gclk_cdata[bit_start_pos+offset].g3 = (mask & g_data ) ? 1:0   ;       
              global_buffer_gclk_cdata[bit_start_pos+offset].b3 = (mask & b_data ) ? 1:0   ;       
              offset++;
            }
        } // rgb1
        break;

        case 4:
        {
            while (subpixel_colour_bit > 0) { // shift out MSB first per the documentation.
              subpixel_colour_bit--;  // start from 7
              uint8_t mask = 1 << subpixel_colour_bit;

              global_buffer_gclk_cdata[bit_start_pos+offset].r4 = (mask & r_data ) ? 1:0   ;       
              global_buffer_gclk_cdata[bit_start_pos+offset].g4 = (mask & g_data ) ? 1:0   ;       
              global_buffer_gclk_cdata[bit_start_pos+offset].b4 = (mask & b_data ) ? 1:0   ;       
              offset++;
            }
        } // rgb1                       
        break;

      } // end switch

#ifdef USE_PSRAM          
        Cache_WriteBack_Addr((uint32_t) &global_buffer_gclk_cdata[bit_start_pos], sizeof(DMA_DATA_TYPE)*8); // Ensure written to PSRAM, needs to be a &reference[x].byteX !                     
#endif        
               
      // We assumpt bit_start_postions we don't touch should be cleared, are.

    }  // mbi_set_pixel  


/*
    IRAM_ATTR void mbi_update()
    {

      dma_buffer_sent = false;

      // set vsync to point back to greyscale buffer before we begin
      dma_ll_vsync[0].empty = (uintptr_t)&dma_ll_gclk_cdata[0]; // back to output data

      // set end of greyscale / gclk buffer to point to start of vsync 
      dma_ll_gclk_cdata[gclk_cdata_last_dma_ll_desc_pos].empty = (uintptr_t)&dma_ll_vsync[0]; // back to output data

      while (!dma_buffer_sent);

      // When the interrupt triggers that the vsync occured, set it so the greyscale buffer loops on itself
      // until such time a vsync is required again.
      // This is done within the interrupt service routine

    } // end   
    */ 

    IRAM_ATTR void mbi_update()
    {


      // Can't get it to work, vsync always happening now...

/*
      dma_buffer_sent = false;

      // set vsync to point back to greyscale buffer before we begin
      dma_ll_vsync[0].empty = (uintptr_t)&dma_ll_gclk_cdata[0]; // back to output data

      // set end of greyscale / gclk buffer to point to start of vsync 
      dma_ll_gclk_cdata[gclk_cdata_last_dma_ll_desc_pos].empty = (uintptr_t)&dma_ll_vsync[0]; // back to output data

      while (!dma_buffer_sent);

      // When the interrupt triggers that the vsync occured, set it so the greyscale buffer loops on itself
      // until such time a vsync is required again.
      // This is done within the interrupt service routine
*/

    } // end     

    // Do it all
    esp_err_t mbi_start()
    {
        ESP_ERROR_CHECK(i2s_lcd_mode_setup());

        i2s_dma_buff_allocate();
        mbi_load_inital_dma_data();
        mbi_send_config();
        mbi_start_output_loop();        

        return ESP_OK;
    }


