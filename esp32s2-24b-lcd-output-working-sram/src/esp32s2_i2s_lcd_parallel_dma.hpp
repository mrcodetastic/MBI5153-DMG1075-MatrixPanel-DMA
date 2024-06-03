#pragma once

#include <string.h> // memcpy
#include <algorithm>
#include <stdbool.h>

#include <sys/types.h>
#include <freertos/FreeRTOS.h>
//#include <driver/i2s.h>
#include <rom/lldesc.h>
#include <rom/gpio.h>
#if (ESP_IDF_VERSION_MAJOR == 5)
#include <driver/i2s_types.h> //includes struct and reg
#else
#include <driver/i2s.h>
#include <soc/i2s_struct.h>
#endif

#include <soc/i2s_periph.h> //includes struct and reg

#define DMA_MAX (4096-4)

// The type used for this SoC
#define HUB75_DMA_DESCRIPTOR_T lldesc_t

#if defined (CONFIG_IDF_TARGET_ESP32S2)   
#define ESP32_I2S_DEVICE I2S_NUM_0	
#else
#pragma error "Not supported."
#endif	


#define DMA_DATA_TYPE uint8_t

//----------------------------------------------------------------------------

 struct config_t
  {
    // Default values
    uint32_t bus_freq     = 2*1000*1000;
    int8_t pin_wr         = -1; // 
    bool   invert_pclk    = false;
    int8_t parallel_width = 24;

    union
    {
      int8_t pin_data[24];
      struct
      {
        int8_t pin_d0;
        int8_t pin_d1;
        int8_t pin_d2;
        int8_t pin_d3;
        int8_t pin_d4;
        int8_t pin_d5;
        int8_t pin_d6;
        int8_t pin_d7;
        int8_t pin_d8;
        int8_t pin_d9;
        int8_t pin_d10;
        int8_t pin_d11;
        int8_t pin_d12;
        int8_t pin_d13;
        int8_t pin_d14;
        int8_t pin_d15;
        int8_t pin_d16;          
        int8_t pin_d17;               
        int8_t pin_d18;                         
        int8_t pin_d19;                                   
        int8_t pin_d20;                                             
        int8_t pin_d21;                                                       
        int8_t pin_d22;                                                                 
        int8_t pin_d23;                                                                           
      };
    };
  };

//----------------------------------------------------------------------------

    void IRAM_ATTR irq_hndlr(void* arg);
    i2s_dev_t* getDev();

    esp_err_t dma_config();
    esp_err_t i2s_lcd_setup_v2(config_t& _cfg);
    esp_err_t dma_allocate_v2(config_t& _cfg);
    esp_err_t dma_start_v2();
