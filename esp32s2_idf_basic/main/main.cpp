/******************************************************************************************
 * @file        main.cpp
 * @author      github.com/mrcodetastic
 * @date        2024
 * @brief       ESP32-S2 implementation for a MBI5135 PWM chip based LED Matrix Panel
 ******************************************************************************************/


/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "rom/cache.h"

#include "esp_timer.h"
#include "rtc.h"
#include "esp32s2/rom/rtc.h"

#include <cmath>

#include "esp32s2_i2s_lcd_24bit_parallel_dma.hpp"
#include "app_constants.h"  

#define PI 3.14159265358979323846

static const char *TAG = "example";

static uint8_t s_led_state = 0;

// the number of the LED pin
const gpio_num_t ledPin = GPIO_NUM_15;  // 16 corresponds to GPIO15

TaskHandle_t myTaskHandle1 = NULL;
TaskHandle_t myTaskHandle2 = NULL;
TaskHandle_t myTaskhandle3 = NULL;


static float angle = 0.0f;
static uint16_t r,g,b;

static void blink_led(void) {
    s_led_state = !s_led_state;
    gpio_set_level(ledPin, s_led_state);
}

void output_task(void *arg) {
    while (1) {

      blink_led();
      printf("Free internal SRAM heap available: %zu bytes\n", heap_caps_get_free_size( MALLOC_CAP_INTERNAL ));
      printf("I2S DMA out_eof interrupt count is: %d\n", get_interrupt_count());

 
      // If you don't have the delay, the main core thead will never get time again.
      for (int i = 10; i >= 0; i--) { // wait 10 seconds
          vTaskDelay(1000 / portTICK_PERIOD_MS);
      }

      fflush(stdout);
    }
} // end output_task


void hsvToRgb(float h, float s, float v, uint16_t &ret_r, uint16_t &ret_g, uint16_t &ret_b) {
    float r = 0, g = 0, b = 0;
    int i = (int)(h * 6);
    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6) {
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        case 5: r = v, g = p, b = q; break;
    }

    ret_r = (uint16_t)(r * 255);
    ret_g = (uint16_t)(g * 255);
    ret_b = (uint16_t)(b * 255);
}

unsigned long millis() {
  return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

unsigned long lastTime = 0;
unsigned long currentTime = 0;
int frame_count = 0;

void graphics_task(void *arg) {

  while (1) {

    currentTime = millis();

    frame_count++;    


    if ((currentTime - lastTime) > 1000)
    {
        printf("Frame Rate is: %d\n", frame_count);

        frame_count = 0;
        lastTime = currentTime;

    }       

    for (int y = 0; y < PANEL_MBI_RES_Y; y++) {
        for (int x = 0; x < PANEL_MBI_RES_X; x++) {
            float dx = x - PANEL_MBI_RES_X / 2;
            float dy = y - PANEL_MBI_RES_Y / 2;
            float theta = atan2(dy, dx) + angle;
            float hue = fmod((theta / (2 * PI)) + 1.0f, 1.0f);
            hsvToRgb(hue, 1.0f, 1.0f,r,g,b);
            mbi_set_pixel(x, y, r,g,b);
        }
    }

    mbi_update();
    //mbi_clear(); // as this is a fullscreen reload, no need to clear

    angle += 0.1f;
  } // while


} // graphics_task



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


void spinning_cube_task(void *arg) {

    while (1) {
      
      currentTime = millis();

      if ((currentTime - lastTime) > 1000)
      {
          printf("Frame Rate is: %d\n", frame_count);

          frame_count = 0;
          lastTime = currentTime;

      }       

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


        frame_count++;    
        draw_wireframe();
        mbi_update();
        vTaskDelay(20 / portTICK_PERIOD_MS);              
        mbi_clear();        
    } // draw wireframe     
       

  } // end while

} // end cube task


/************************************************************************/
void rgb_loop_task(void *arg) {

    static int rgb_mode = 0;

    while (1) {
      /*
      currentTime = millis();

      if ((currentTime - lastTime) > 1000)
      {
          printf("Frame Rate is: %d\n", frame_count);

          frame_count = 0;
          lastTime = currentTime;

      } 
      */      

      int r = 0;
      int g = 0;
      int b = 0;
      if (rgb_mode == 0 )
      {
        r = 255;
        rgb_mode++;        
      }
      else if (rgb_mode == 1)
      {
        g = 255;
        rgb_mode++;        
      }
      else if (rgb_mode == 2)
      {
        b = 255;
        rgb_mode++;
      }
     else if (rgb_mode == 3) // black
      {
        rgb_mode = 0;
      }      



      for (int y = 0; y < PANEL_MBI_RES_Y; y++) {
              for (int x = 0; x < PANEL_MBI_RES_X; x++) {                
                  mbi_set_pixel(x, y, r,g,b);
              }
          }

        mbi_update();
        vTaskDelay(1000 / portTICK_PERIOD_MS);              
        //mbi_clear();  

    } // draw wireframe     
       
} // end cube task


/************************************************************************/

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(ledPin);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(ledPin, GPIO_MODE_OUTPUT);

    blink_led();    

    for (int delaysec = 3; delaysec > 0; delaysec--)
    {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      ESP_LOGI("app_main", "Starting in %d...", delaysec);
    }

    ESP_LOGI("app_main", "Calling mbi_start()");
    mbi_start();

    ESP_LOGI("app_main", "Creating task to do regular update.");        
    xTaskCreate(output_task, "Output_Task", 4096, NULL, 10, &myTaskHandle1);


    ESP_LOGI("app_main", "Creating graphics task.");        
    //xTaskCreate(graphics_task, "Graphics_Task", 4096, NULL, 10, &myTaskHandle2);
    xTaskCreate(spinning_cube_task, "Graphics_Task", 4096, NULL, 10, &myTaskHandle2);

    //xTaskCreate(rgb_loop_task, "Graphics_Task", 4096, NULL, 10, &myTaskHandle2);




}