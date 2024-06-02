#pragma once

#if defined (ESP_PLATFORM)

 #include <sdkconfig.h>

 #if defined (CONFIG_IDF_TARGET_ESP32S2)

  //#pragma message "Compiling for ESP32-S2"
  #include "esp32/esp32_i2s_parallel_dma.hpp"  
  #include "esp32s2/esp32s2-default-pins.hpp"  

 #elif defined (CONFIG_IDF_TARGET_ESP32) || defined(ESP32)

  // Assume an ESP32 (the original 2015 version)
  // Same include as ESP32S3  
  //#pragma message "Compiling for original ESP32 (released 2016)"  
  
  #define ESP32_THE_ORIG 1	
  //#include "esp32/esp32_i2s_parallel_dma.hpp"
  //#include "esp32/esp32_i2s_parallel_dma.h"
  #include "esp32/esp32_i2s_parallel_dma.hpp"  
  #include "esp32/esp32-default-pins.hpp"

 #else
    #error "Unsupported platform."
  
 #endif


#endif

