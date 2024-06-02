/*----------------------------------------------------------------------------/
  Lovyan GFX - Graphics library for embedded devices.

Original Source:
 https://github.com/lovyan03/LovyanGFX/

Licence:
 [FreeBSD](https://github.com/lovyan03/LovyanGFX/blob/master/license.txt)

Author:
 [lovyan03](https://twitter.com/lovyan03)

Contributors:
 [ciniml](https://github.com/ciniml)
 [mongonta0716](https://github.com/mongonta0716)
 [tobozo](https://github.com/tobozo)

Modified heavily for the ESP32 HUB75 DMA library by:
 [mrfaptastic](https://github.com/mrfaptastic)
  
/----------------------------------------------------------------------------*/
#include <sdkconfig.h>
#if defined (CONFIG_IDF_TARGET_ESP32) || defined (CONFIG_IDF_TARGET_ESP32S2)

#include "esp32_i2s_parallel_dma.hpp"

#if defined (CONFIG_IDF_TARGET_ESP32S2)
  #pragma message "Compiling for ESP32-S2"
#else
  #pragma message "Compiling for original ESP32 (released 2016)"  
#endif

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

static const char *TAG = "cam_hal";


//lldesc_t *dma;
//uint8_t  *dma_buffer; 
cam_frame_t *frames;

int frame_pos = 0;


	volatile bool previousBufferFree = true;

	static void IRAM_ATTR i2s_isr(void* arg) {

		// From original Sprite_TM Code
		//REG_WRITE(I2S_INT_CLR_REG(1), (REG_READ(I2S_INT_RAW_REG(1)) & 0xffffffc0) | 0x3f);
		
		// Clear flag so we can get retriggered
		SET_PERI_REG_BITS(I2S_INT_CLR_REG(ESP32_I2S_DEVICE), I2S_OUT_EOF_INT_CLR_V, 1, I2S_OUT_EOF_INT_CLR_S);                      
		
		// at this point, the previously active buffer is free, go ahead and write to it
		previousBufferFree = true;
	}

	bool DRAM_ATTR i2s_parallel_is_previous_buffer_free() {
		return previousBufferFree;
	}


	// Static
	i2s_dev_t* getDev()
	{
		  return &I2S0;
	}

	// Static
	void _gpio_pin_init(int pin)
	{
		if (pin >= 0)
		{
		  gpio_pad_select_gpio(pin);
		  //gpio_hi(pin);
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

#define CAM_CHECK(a, str, ret) if (!(a)) {                                          \
        ESP_LOGE(TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str);                    \
        return (ret);                                                               \
        }

#define CAM_CHECK_GOTO(a, str, lab) if (!(a)) {                                     \
        ESP_LOGE(TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str);                    \
        goto lab;                                                                   \
        }

#define LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE  (4092)


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

uint8_t* bufferx;
 
static lldesc_t * allocate_dma_descriptors(uint32_t count, uint16_t size, uint8_t * buffer)
{
    lldesc_t *dma = (lldesc_t *)heap_caps_malloc(count * sizeof(lldesc_t), MALLOC_CAP_DMA);
    if (dma == NULL) {
        return dma;
    }

    for (int x = 0; x < count; x++) {

        // Need to decrement data and save

        dma[x].size = size;
        dma[x].length = size;
        dma[x].sosf = 0;
        dma[x].eof = 0;
        dma[x].owner = 1;
        dma[x].buf = (buffer + size * x);

        if ( ((x + 1) % count) == 0 ) { // last element

           dma[x].empty = (uint32_t)&dma[0];
           ESP_LOGD("allocate_dma_descriptors()", "Linking lldesc_t  pos %d back to post 0 at memory location %08x", x, (uintptr_t)&dma[(x + 1) % count]);      


        } else {
           dma[x].empty = (uint32_t)&dma[(x + 1) % count];
           ESP_LOGD("allocate_dma_descriptors()", "Linking lldesc_t  pos %d to pos %d at memory location %08x", x, (x+1), (uintptr_t)&dma[(x + 1) % count]);      
        }
        // dma[x].empty = (uint32_t)&dma[(x + 1) % count];
    }
    return dma;
}



static lldesc_t * allocate_dma_descriptors_gb(uint32_t count, uint16_t payload_size, uint8_t * buffer)
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
        dma[n].buf = (buffer);

        payload_size -= dmachunklen;        
        buffer += dmachunklen;  

        if ( n == (count-1) ) { // last element
           dma[n].empty = (uint32_t)&dma[0];
           ESP_LOGD("allocate_dma_descriptors()", "Linking lldesc_t  pos %d back to pos 0", n);      

        } else {
           dma[n].empty = (uint32_t)&dma[(n + 1) % count];
           ESP_LOGD("allocate_dma_descriptors()", "Linking lldesc_t  pos %d to pos %d at memory location %08x", n, (n+1), (uintptr_t)&dma[(n + 1) % count]);      
        }

       n++;


    }

    //dma[n - 1].dw0.suc_eof = 1; //Mark last DMA desc as end of stream.
    //dma[n - 1].next = NULL;


    return dma;
}


int ll_cam_get_dma_align()
{

    ESP_LOGD("ll_cam_get_dma_align()", "ext_mem_bk_size val is %d", I2S0.lc_conf.ext_mem_bk_size);         

    //return 16; //16 << I2S0.lc_conf.ext_mem_bk_size;   
    //return 64;//16 << I2S0.lc_conf.ext_mem_bk_size;   
    //return 1024; //16 << I2S0.lc_conf.ext_mem_bk_size;    

    return 32;

}


static int ll_desc_get_required_num(uint32_t bytes_len)
{
    int ll_desc_required = (bytes_len + DMA_MAX - 1) / DMA_MAX;
    ESP_LOGD("ll_desc_get_required_num()", "Gunna need %d dma lldesc's", ll_desc_required);      

    return ll_desc_required;

}

esp_err_t dma_config()
{

    uint8_t pattern_len = 4;
                                //  
    uint8_t pattern[4] = { 0b00000000, 0x00, 0x00, 0x00};

    //size_t fb_size = 6000; // arbitrary 3 (for 24 bit) * 2000
    size_t fb_size = 4*32; // arbitrary 3 byes * 30 pulses = 90
    size_t payload_size = fb_size;

    int dma_node_buffer_size = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
    int dma_node_cnt = ll_desc_get_required_num(fb_size); // Number of DMA nodes  8000 / 4092

    //dma_buffer = NULL;
    //dma = NULL;

    int test_buff_size = 32*1024;

    int frame_cnt = 1; // one buffer

    frames = (cam_frame_t *)heap_caps_aligned_calloc(alignof(cam_frame_t), 1, frame_cnt * sizeof(cam_frame_t), MALLOC_CAP_DEFAULT);
    CAM_CHECK(frames != NULL, "frames malloc failed", ESP_FAIL);

    int dma_align = ll_cam_get_dma_align();
    
    /* Allocate memory for frame buffer */
    size_t alloc_size = fb_size * sizeof(uint8_t) + dma_align;

    bufferx = (uint8_t *)heap_caps_aligned_calloc(dma_align, 1, test_buff_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM );
    if (bufferx == NULL)  {
             ESP_LOGE(TAG, "xxxx Frame buffer malloc failed.");
    } else
    {
          //memset(bufferx, 0x00, 32*1024); // clean it up

          // load a pattern
          int bytenum = 0;
          while (bytenum < (test_buff_size - pattern_len)) {

             memcpy(bufferx + bytenum, pattern, pattern_len);

             bytenum += pattern_len;

          }
    }

    uint32_t _caps = MALLOC_CAP_8BIT;

    // _caps |= MALLOC_CAP_INTERNAL;
    _caps |= MALLOC_CAP_SPIRAM;




    for (int x = 0; x < frame_cnt; x++) 
    { //0 , 1
        frames[x].dma = NULL;
        frames[x].fb_offset = 0;
        frames[x].en = 0;
        ESP_LOGI(TAG, "Allocating %d Byte frame buffer in %s", alloc_size, _caps & MALLOC_CAP_SPIRAM ? "PSRAM" : "OnBoard RAM");
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)

        // In IDF v4.2 and earlier, memory returned by heap_caps_aligned_alloc must be freed using heap_caps_aligned_free.
        // And heap_caps_aligned_free is deprecated on v4.3.
        frames[x].fb.buf = (uint8_t *)heap_caps_aligned_alloc(16, alloc_size, _caps);
#else
        frames[x].fb.buf = (uint8_t *)heap_caps_malloc(alloc_size, _caps);
#endif
        if (frames[x].fb.buf == NULL) {
                ESP_LOGE(TAG, "Frame buffer malloc failed.");
        }

        //CAM_CHECK(cam_obj->frames[x].fb.buf != NULL, "frame buffer malloc failed", ESP_FAIL);
        //if (cam_obj->psram_mode) {
        if (true)
        {
            //align PSRAM buffer. TODO: save the offset so proper address can be freed later
            frames[x].fb_offset = dma_align - ((uint32_t)frames[x].fb.buf & (dma_align - 1));
            frames[x].fb.buf += frames[x].fb_offset;
            ESP_LOGI(TAG, "Frame[%d]: Offset: %u, Addr: 0x%08X", x, frames[x].fb_offset, (unsigned) frames[x].fb.buf);
            //frames[x].dma = allocate_dma_descriptors(dma_node_cnt, cam_obj->dma_node_buffer_size, frames[x].fb.buf);
           // frames[x].dma = allocate_dma_descriptors(dma_node_cnt, dma_node_buffer_size, frames[x].fb.buf);

            //frames[x].dma = allocate_dma_descriptors_gb(dma_node_cnt, payload_size, frames[x].fb.buf);

            // Test
            size_t offset2 = dma_align - ((uint32_t)bufferx & (dma_align - 1));
            //frames[x].dma = allocate_dma_descriptors_gb(dma_node_cnt, payload_size, (bufferx+offset2));
            frames[x].dma = allocate_dma_descriptors_gb(ll_desc_get_required_num(test_buff_size), test_buff_size, (bufferx+offset2));
            

            CAM_CHECK(frames[x].dma != NULL, "frame dma malloc failed", ESP_FAIL);
        }
        frames[x].en = 1;
    }

    // Clear the memory area.
    memset(frames[frame_pos].fb.buf, 0xff, fb_size); // clean it up


/*
    int flip = 0;
    for (int i = 0; i < alloc_size; i++)
    {
      if ( (i % 3) == 0){ flip = !flip; }

      frames[0].fb.buf[i] = (flip == 0) ? 0xFF:0x0;

    }
    */

    return ESP_OK;
}

  

  void Bus_Parallel16::config(const config_t& cfg)
  {
      ESP_LOGI("ESP32/S2", "Performing config for ESP32 or ESP32-S2");
      _cfg = cfg;
      _dev = getDev();
  }
 
 bool Bus_Parallel16::init(void) // The big one that gets everything setup.
 {
  /*
    if(_cfg.parallel_width < 8 || _cfg.parallel_width > 24) {
      return false;
    }
   */

    auto dev = _dev;
    volatile int iomux_signal_base;
    volatile int iomux_clock;
    int irq_source;

    // Initialize I2S0 peripheral
    if (ESP32_I2S_DEVICE == I2S_NUM_0) 
    {
        periph_module_reset(PERIPH_I2S0_MODULE);
        periph_module_enable(PERIPH_I2S0_MODULE);

        iomux_clock = I2S0O_WS_OUT_IDX;
        irq_source = ETS_I2S0_INTR_SOURCE;

        switch(_cfg.parallel_width) {
          case 8:
          case 16:
            iomux_signal_base = I2S0O_DATA_OUT8_IDX;
            break;
          case 24:
            ESP_LOGI(TAG, "Configuring signal base for 24bit");
            iomux_signal_base = I2S0O_DATA_OUT0_IDX;
            break;

          // 32bit parallel not possible in LCD mode

          default:
            return ESP_ERR_INVALID_ARG;
        }
    } 

    // Setup GPIOs
    int bus_width = _cfg.parallel_width;

    // Clock output GPIO setup
    _gpio_pin_init(_cfg.pin_rd); // not used
    _gpio_pin_init(_cfg.pin_wr); // clock
    _gpio_pin_init(_cfg.pin_rs); // not used

    // Data output GPIO setup
    int8_t* pins = _cfg.pin_data;  

    for(int i = 0; i < bus_width; i++) 
     _gpio_pin_init(pins[i]);

    // Route clock signal to clock pin (can route to two pins if we want)
    gpio_matrix_out(_cfg.pin_wr, iomux_clock, _cfg.invert_pclk, 0); // inverst clock if required
    //gpio_matrix_out(GPIO_NUM_13, iomux_clock, _cfg.invert_pclk, 0); // inverst clock if required two pins

    for (size_t i = 0; i < bus_width; i++) {

      if (pins[i] >= 0) {
        gpio_matrix_out(pins[i], iomux_signal_base + i, false, false);
      }
    }

    ////////////////////////////// Clock configuration //////////////////////////////



    // unsigned int freq 		= (_cfg.bus_freq);

    unsigned int freq = 2*1000*1000;

    ESP_LOGD("ESP32/S2", "Requested output clock frequency: %u Mhz",  (unsigned int)(freq/1000000));   		
	
	// What is the current CPU frequency?

  
	
		// Right shift (>> 1) and divide 160mhz in half to 80Mhz for the calc due to the fact 
		// that later we must have tx_bck_div_num = 2  for both esp32 and esp32-s2

		//static uint32_t pll_160M_clock_d2 = 160 * 1000 * 1000 >> 1;

		// I2S_CLKM_DIV_NUM 2=40MHz  /  3=27MHz  /  4=20MHz  /  5=16MHz  /  8=10MHz  /  10=8MHz
		//auto _div_num = std::min(255u, 1 + ((pll_160M_clock_d2) / (1 + freq)));
		 unsigned int _div_num = (unsigned int) (160000000L / freq / i2s_parallel_get_memory_width(ESP32_I2S_DEVICE, bus_width)); 


		ESP_LOGD("ESP32", "i2s pll_160M_clock_d2 clkm_div_num is: %u", _div_num);    		

		// I2S_CLK_SEL Set this bit to select I2S module clock source. 
		// 0: No clock. 1: APLL_CLK. 2: PLL_160M_CLK. 3: No clock. (R/W)
		dev->clkm_conf.clk_sel = 2; 
		dev->clkm_conf.clkm_div_a = 1;      // Clock denominator 			
		dev->clkm_conf.clkm_div_b = 0;      // Clock numerator
		dev->clkm_conf.clkm_div_num = _div_num;				
    //dev->clkm_conf.clkm_div_num = (160/4);				// 4mhz
		dev->clkm_conf.clk_en  = 1;
		
		// Calc
		unsigned int output_freq = (unsigned int)(160000000L/dev->clkm_conf.clkm_div_num);		

    
    ////////////////////////////// END CLOCK CONFIGURATION /////////////////////////////////

    // I2S conf2 reg
    dev->conf2.val = 0;
    dev->conf2.lcd_en = 1;
    // I2S conf reg
    dev->conf.val = 0;    // clear tx_slave_mod


    // Setup i2s clock
    //dev->sample_rate_conf.val = 0;
    
    // Third stage config, width of data to be written to IO (I think this should always be the actual data width?)
    //dev->sample_rate_conf.rx_bits_mod = bus_width;
    dev->sample_rate_conf.tx_bits_mod = bus_width;

		ESP_LOGD("ESP32", "Bus width is: %d", bus_width);      

    
    // Serial clock 
	// ESP32 and ESP32-S2 TRM clearly say that "Note that I2S_TX_BCK_DIV_NUM[5:0] must not be configured as 1."
    dev->sample_rate_conf.rx_bck_div_num = 2;
    dev->sample_rate_conf.tx_bck_div_num = 2;
        
    
    /* 4. Set I2S_TX_DMA_EQUAL, then clear I2S_TX_RIGHT_FIRST, I2S_LCD_TX_WRX2_EN, I2S_LCD_TX_SDX2_EN, and I2S_TX_CHAN_MOD[2:0]. */
    
    dev->conf.tx_dma_equal=1;  // esp32-s2 only
    //dev->conf.tx_right_first=0;
    dev->conf2.lcd_tx_wrx2_en=0; 
    dev->conf2.lcd_tx_sdx2_en=0;    

    //dev->conf_chan.tx_chan_mod = 3;

    //dev->conf.tx_big_endian = 0;  /// ????????

    //dev->conf.pre_req_en=1;    // esp32-s2 only - enable I2S to prepare data earlier? wtf?

    // Now start setting up DMA FIFO
    dev->fifo_conf.val = 0;  
    //dev->fifo_conf.rx_data_num = 32; // Thresholds. 
    dev->fifo_conf.tx_data_num = 32;  
    dev->fifo_conf.dscr_en     = 1;  
    dev->fifo_conf.tx_fifo_mod_force_en = 1;

    //dev->fifo_conf.tx_24msb_en = 1; // high or low 24 bits?

/*
  #if defined (CONFIG_IDF_TARGET_ESP32S2)  

    // Not really described for non-pcm modes, although datasheet states it should be set correctly even for LCD mode
    // First stage config. Configures how data is loaded into fifo
    if(_cfg.parallel_width == 24) { 

    ESP_LOGI("ESP32/S2", "Configuring for ESP32S2 24 bit output.");

      // Mode 0, single 32-bit channel, linear 32 bit load to fifo
        dev->fifo_conf.tx_fifo_mod = 3;

        dev->fifo_conf.tx_24msb_en = 0; // high or low 24 bits?

    } else {
      // Mode 1, single 16-bit channel, load 16 bit sample(*) into fifo and pad to 32 bit with zeros
      // *Actually a 32 bit read where two samples are read at once. Length of fifo must thus still be word-aligned
      dev->fifo_conf.tx_fifo_mod = 1;
    }
  
    // Dictated by ESP32 datasheet
    //dev->fifo_conf.rx_fifo_mod_force_en = 1;
    dev->fifo_conf.tx_fifo_mod_force_en = 1;

  #endif  

    //confirmdev->fifo_conf.tx_fifo_mod = 3;

   // dev->conf.tx_right_first = 1;
  //  dev->conf.tx_msb_shift = 0;
*/


   // Device reset
    //dev->conf.rx_reset=1;
    //dev->conf.rx_reset=0;

    dev->conf.tx_reset=1;
    dev->conf.tx_reset=0;   

    // Reset TX FIFO
    dev->conf.tx_fifo_reset = 1;
    #if defined (CONFIG_IDF_TARGET_ESP32S2)
      while(dev->conf.tx_fifo_reset_st != 0); // esp32-s2 only
    #endif
      dev->conf.tx_fifo_reset = 0;



    // Reset DMA
    //dev->lc_conf.val = 0;
    dev->lc_conf.out_eof_mode = 1;
    dev->lc_conf.in_rst = 1;
    dev->lc_conf.in_rst = 0;
    dev->lc_conf.out_rst = 1;
    dev->lc_conf.out_rst = 0;

    dev->lc_conf.ahbm_fifo_rst = 1;
    dev->lc_conf.ahbm_fifo_rst = 0;    
    
    dev->lc_conf.ahbm_rst = 1;
    dev->lc_conf.ahbm_rst = 0;





    // New
    dev->lc_conf.ext_mem_bk_size = 1; // /*DMA access external memory block size. 0: 16 bytes      1: 32 bytes    2:64 bytes      3:reserved*/
    dev->lc_conf.check_owner = 0;


    dev->in_link.val = 0;      // don't start
    dev->out_link.val = 0;     // don't start out link


    //dev->conf1.val = 0;
    //dev->conf1.tx_stop_en = 0; 
    //dev->conf1.tx_pcm_bypass = 1; 


    //dev->timing.val = 0;


    // If we have double buffering, then allocate an interrupt service routine function
    // that can be used for I2S0/I2S1 created interrupts.

    // Setup I2S Interrupt
    //SET_PERI_REG_BITS(I2S_INT_ENA_REG(ESP32_I2S_DEVICE), I2S_OUT_EOF_INT_ENA_V, 1, I2S_OUT_EOF_INT_ENA_S);

    // Allocate a level 1 intterupt: lowest priority, as ISR isn't urgent and may take a long time to complete
    //esp_intr_alloc(irq_source, (int)(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1), i2s_isr, NULL, NULL);

  

     return true;
  }


  void Bus_Parallel16::release(void)
  {
    if (_dmadesc_a)
    {
      heap_caps_free(_dmadesc_a);
      _dmadesc_a = nullptr;
      _dmadesc_count = 0;
    }

    if (_dmadesc_b)
    {
      heap_caps_free(_dmadesc_b);
      _dmadesc_b = nullptr;
      _dmadesc_count = 0;      
    }
  }

  void Bus_Parallel16::enable_double_dma_desc(void)
  {
    _double_dma_buffer = true;
  }

  // Need this to work for double buffers etc.
  bool Bus_Parallel16::allocate_dma_desc_memory(size_t len)
  {
    if (_dmadesc_a) heap_caps_free(_dmadesc_a); // free all dma descrptios previously
    
    _dmadesc_count = len; 
    _dmadesc_last  = len-1; 

    ESP_LOGI("ESP32/S2", "Allocating memory for %d DMA descriptors.", (int)len);    

    _dmadesc_a= (HUB75_DMA_DESCRIPTOR_T*)heap_caps_aligned_alloc(32, sizeof(HUB75_DMA_DESCRIPTOR_T) * len, MALLOC_CAP_DMA);

    if (_dmadesc_a == nullptr)
    {
      ESP_LOGE("ESP32/S2", "ERROR: Couldn't malloc _dmadesc_a. Not enough memory.");
      return false;
    }


    if (_double_dma_buffer)
    {
      if (_dmadesc_b) heap_caps_free(_dmadesc_b); // free all dma descrptios previously

      ESP_LOGD("ESP32/S2", "Allocating the second buffer (double buffer enabled).");              

      _dmadesc_b = (HUB75_DMA_DESCRIPTOR_T*)heap_caps_aligned_alloc(32,sizeof(HUB75_DMA_DESCRIPTOR_T) * len, MALLOC_CAP_DMA);

      if (_dmadesc_b == nullptr)
      {
        ESP_LOGE("ESP32/S2", "ERROR: Couldn't malloc _dmadesc_b. Not enough memory.");
        _double_dma_buffer = false;
        return false;
      }
    }
    
    _dmadesc_a_idx  = 0;
    _dmadesc_b_idx  = 0;

    ESP_LOGD("ESP32/S2", "Allocating %d bytes of memory for DMA descriptors.", (int)sizeof(HUB75_DMA_DESCRIPTOR_T) * len);       
/*
    // New - Temporary blank descriptor for transitions between DMA buffer
    _dmadesc_blank = (HUB75_DMA_DESCRIPTOR_T*)heap_caps_malloc(sizeof(HUB75_DMA_DESCRIPTOR_T) * 1, MALLOC_CAP_DMA);
    _dmadesc_blank->size     = 1024*2;
    _dmadesc_blank->length   = 1024*2;
    _dmadesc_blank->buf      = (uint8_t*) _blank_data; 
    _dmadesc_blank->eof      = 1;         
    _dmadesc_blank->sosf     = 0;         
    _dmadesc_blank->owner    = 1;         
    _dmadesc_blank->qe.stqe_next = (lldesc_t*) _dmadesc_blank;         
    _dmadesc_blank->offset   = 0;         
*/
    return true;

  }

  void Bus_Parallel16::create_dma_desc_link(void *data, size_t size, bool dmadesc_b)
  {
    static constexpr size_t MAX_DMA_LEN = (4096-4);

    if (size > MAX_DMA_LEN)
    {
      size = MAX_DMA_LEN;
      ESP_LOGW("ESP32/S2", "Creating DMA descriptor which links to payload with size greater than MAX_DMA_LEN!");            
    }

    if ( !dmadesc_b )
    {
      if ( (_dmadesc_a_idx+1) > _dmadesc_count) {
        ESP_LOGE("ESP32/S2", "Attempted to create more DMA descriptors than allocated memory for. Expecting a maximum of %u DMA descriptors", (unsigned int)_dmadesc_count);          
        return;
      }
    }

    volatile lldesc_t *dmadesc;
    volatile lldesc_t *next;
    bool eof = false;
    
    if ( (dmadesc_b == true) ) // for primary buffer
    {
        dmadesc      = &_dmadesc_b[_dmadesc_b_idx];

        next = (_dmadesc_b_idx < (_dmadesc_last) ) ? &_dmadesc_b[_dmadesc_b_idx+1]:_dmadesc_b;       
        eof  = (_dmadesc_b_idx == (_dmadesc_last));
    }
    else
    {
        dmadesc      = &_dmadesc_a[_dmadesc_a_idx];

        // https://stackoverflow.com/questions/47170740/c-negative-array-index
        next = (_dmadesc_a_idx < (_dmadesc_last) ) ? _dmadesc_a + _dmadesc_a_idx+1:_dmadesc_a;       
        eof  = (_dmadesc_a_idx == (_dmadesc_last));
    }

    if ( _dmadesc_a_idx == (_dmadesc_last) ) {
      ESP_LOGW("ESP32/S2", "Creating final DMA descriptor and linking back to 0.");             
    } 

    dmadesc->size     = size;
    dmadesc->length   = size;
    dmadesc->buf      = (uint8_t*) data; 
    dmadesc->eof      = eof;         
    dmadesc->sosf     = 0;         
    dmadesc->owner    = 1;         
    dmadesc->qe.stqe_next = (lldesc_t*) next;         
    dmadesc->offset   = 0;       

    if ( (dmadesc_b == true) ) { // for primary buffer
      _dmadesc_b_idx++; 
    } else {
      _dmadesc_a_idx++; 
    }
  
  } // end create_dma_desc_link

  void Bus_Parallel16::dma_transfer_start()
  {
    auto dev = _dev;
   
   
    // Configure DMA burst mode
    dev->lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN;
/*
    // Set address of DMA descriptor, start with buffer 0 / 'a'
    dev->out_link.addr = (uint32_t) _dmadesc_a;
  
  // Start DMA operation
    dev->out_link.stop  = 0; 
    dev->out_link.start = 1;
    
    dev->conf.tx_start  = 1;
*/

/*
  // from ll_cam.c in the esp32-camera example
  
  I2S0.rx_eof_num = cam->dma_half_buffer_size; // Ping pong operation
    if (!cam->psram_mode) {
        I2S0.in_link.addr = ((uint32_t)&cam->dma[0]) & 0xfffff;
    } else {
        I2S0.in_link.addr = ((uint32_t)&cam->frames[frame_pos].dma[0]) & 0xfffff;
    }
*/

    dev->out_link.addr = ((uint32_t)&frames[frame_pos].dma[0]) & 0xfffff;

    dev->out_link.start = 1;
    
    dev->conf.tx_start  = 1;    


   
  } // end 
  

  void Bus_Parallel16::dma_transfer_stop()
  {
     auto dev = _dev;
     
    // Stop all ongoing DMA operations
    dev->out_link.stop = 1;
    dev->out_link.start = 0;
    dev->conf.tx_start = 0;
        
  } // end   


  void Bus_Parallel16::flip_dma_output_buffer(int buffer_id) // pass by reference so we can change in main matrixpanel class
  {
	  
      // Setup interrupt handler which is focussed only on the (page 322 of Tech. Ref. Manual)
      // "I2S_OUT_EOF_INT: Triggered when rxlink has finished sending a packet" (when dma linked list with eof = 1 is hit)
  	  
      if ( buffer_id == 1) { 

	      _dmadesc_a[_dmadesc_last].qe.stqe_next = &_dmadesc_b[0]; // Start sending out _dmadesc_b (or buffer 1)

        //fix _dmadesc_ loop issue #407
        //need to connect the up comming _dmadesc_ not the old one
        _dmadesc_b[_dmadesc_last].qe.stqe_next = &_dmadesc_b[0];
		      
      } else { 
	      
        _dmadesc_b[_dmadesc_last].qe.stqe_next = &_dmadesc_a[0]; 
        _dmadesc_a[_dmadesc_last].qe.stqe_next = &_dmadesc_a[0];
		
      }

      previousBufferFree = false;  
      //while (i2s_parallel_is_previous_buffer_free() == false) {}      
      while (!previousBufferFree);

           


  } // end flip



#endif
