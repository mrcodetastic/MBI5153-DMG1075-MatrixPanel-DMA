#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "sdkconfig.h"
#include <esp_log.h>
#include "esp_task_wdt.h"
#include "main.hpp"
#include "ewaste_MBI5153.hpp"
#include "esp32s3_peripheral.h"
#include "test_pattern.h"
#include "dma_data.h"

#define GCLK_ADDR_MODE_DMA 1

// Selected mode
#define GCLK_ADDR_MODE GCLK_ADDR_MODE_DMA


static const char *TAG  = "app_main";
volatile int refresh    = 0;
volatile int image      = 0;

bool alloc_dma_data_buffer() 
{ 
    dma_gpio_size        = sizeof(dma_data_nodt);
    ESP_LOGI("I2S-DMA", "Size of DMA data we need to use to drive is %u.", dma_gpio_size);  
    
    // Defined in main.hpp
    dma_gpio_data = (ESP32_I2S_DMA_STORAGE_TYPE *)heap_caps_malloc(dma_gpio_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

    if (dma_gpio_data == nullptr) 
    {
        ESP_LOGE("I2S-DMA", "Could not malloc space..");
        return false;
        // TODO: should we release all previous rowBitStructs here???
    }  

    ESP_LOGE("I2S-DMA", "Copying bootstrap DMA data across.");                
    memcpy(dma_gpio_data, dma_data_nodt, dma_gpio_size);    // Using the same data for now.

    return true;
} // end dma alloc

bool configure_dma_gclk()
{  
    if ( !alloc_dma_data_buffer() )
    {
        ESP_LOGI("I2S-DMA", "Failed to allocate DMA memory.");          
        return false;
    }

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



// Start the App
void setup(void)
{
    Serial.begin(112500);

    ESP_LOGD(TAG, "Configure GPIOs");     
       
    setup_gpio_dir();
    setup_gpio_output();    
    
    esp_task_wdt_deinit();
      

 
    #if (GCLK_ADDR_MODE == GCLK_ADDR_MODE_DMA)

        ESP_LOGD(TAG, "Setup MBI5153 with DMA"); 

        configure_dma_gclk();  
        load_dma_data_buffer(true);  
        dma_bus.dma_transfer_start();  // start gclk + addr toggle
      
        // Confiure register1
        mbi_soft_reset();    
        mbi_pre_active(); // must      
        mbi_configuration(ghost_elimination_ON,(PANEL_SCAN_LINES-1),gray_scale_14,gclk_multiplier_OFF,current_1);  

        // Configure register 2
        mbi_pre_active(); // must          
        mbi_configuration2();    
        
    #endif

}


int row = 0;
unsigned long  last_display  = 0;
void loop()
{

    if (refresh)
    {

      if (image > 2) image = 0;
      
      switch (image)
      {
          case 0: mbi_set_frame_lvgl_rgb(test_pattern_1); // house
                  break;
          case 1: mbi_set_frame_lvgl_rgb(test_pattern_2); // lines
                  break;
          case 2: mbi_set_frame_lvgl_rgb(test_pattern_3); // text
                  break;
      }

        dma_bus.dma_transfer_stop();          
        mbi_v_sync(); 
        dma_bus.dma_transfer_start();

        image++;
        refresh = 0;
    }


    if ((millis() - last_display) > 1000)  {
    
        last_display = millis();
  
        refresh = 1;

        ESP_LOGI("Main", "Requeseted refresh with image %d", image);

    }    
}

