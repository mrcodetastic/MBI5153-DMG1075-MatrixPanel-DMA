#include <Arduino.h>
#include <esp_log.h>

#define PANEL_RES_X 80    // Number of pixels wide of each INDIVIDUAL panel module. 
#define PANEL_RES_Y 80     // Number of pixels tall of each INDIVIDUAL panel module.
#define PANEL_CHAIN 1      // Total number of panels chained one to another
#define USE_FLOATHACK      // To boost float performance, comment if this doesn't work. 

#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

MatrixPanel_I2S_DMA *dma_display = nullptr;


uint16_t myBLACK = dma_display->color565(0, 0, 0);
uint16_t myWHITE = dma_display->color565(255, 255, 255);
uint16_t myRED = dma_display->color565(255, 0, 0);
uint16_t myGREEN = dma_display->color565(0, 255, 0);
uint16_t myBLUE = dma_display->color565(0, 0, 255);



// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
// From: https://gist.github.com/davidegironi/3144efdc6d67e5df55438cc3cba613c8
uint16_t colorWheel(uint8_t pos) {
  if(pos < 85) {
    return dma_display->color565(pos * 3, 255 - pos * 3, 0);
  } else if(pos < 170) {
    pos -= 85;
    return dma_display->color565(255 - pos * 3, 0, pos * 3);
  } else {
    pos -= 170;
    return dma_display->color565(0, pos * 3, 255 - pos * 3);
  }
}

void drawText(int colorWheelOffset)
{

  // Test the bandwidth of drawing every pixel in the memory buffer 16 times each iteration.
  for (int x = 0; x < 16; x++)
  {
    for (int i = 0; i < PANEL_RES_X; i++) {
      for (int j = 0; i < PANEL_RES_X; i++){
        dma_display->drawPixel(i,j, myBLACK);
      }
    }
  }
  
  // draw text with a rotating colour
  dma_display->setTextSize(1);     // size 1 == 8 pixels high
  dma_display->setTextWrap(false); // Don't wrap at end of line - will do ourselves

  dma_display->setCursor(5, 0);    // start at top left, with 8 pixel of spacing
  uint8_t w = 0;
  const char *str = "ESP32 DMA";
  for (w=0; w<strlen(str); w++) {
    dma_display->setTextColor(colorWheel((w*32)+colorWheelOffset));
    dma_display->print(str[w]);
  }

  dma_display->println();
  dma_display->print(" ");
  for (w=9; w<18; w++) {
    dma_display->setTextColor(colorWheel((w*32)+colorWheelOffset));
    dma_display->print("*");
  }
  
  dma_display->println();

  dma_display->setTextColor(dma_display->color444(15,15,15));
  dma_display->println("LED MATRIX!");

  // print each letter with a fixed rainbow color
  dma_display->setTextColor(dma_display->color444(0,8,15));
  dma_display->print('3');
  dma_display->setTextColor(dma_display->color444(15,4,0));
  dma_display->print('2');
  dma_display->setTextColor(dma_display->color444(15,15,0));
  dma_display->print('x');
  dma_display->setTextColor(dma_display->color444(8,15,0));
  dma_display->print('6');
  dma_display->setTextColor(dma_display->color444(8,0,15));
  dma_display->print('4');

  // Jump a half character
  dma_display->setCursor(34, 24);
  dma_display->setTextColor(dma_display->color444(0,15,15));
  dma_display->print("*");
  dma_display->setTextColor(dma_display->color444(15,0,0));
  dma_display->print('R');
  dma_display->setTextColor(dma_display->color444(0,15,0));
  dma_display->print('G');
  dma_display->setTextColor(dma_display->color444(0,0,15));
  dma_display->print("B");
  dma_display->setTextColor(dma_display->color444(15,0,8));
  dma_display->println("*");

}

uint64_t lastMillis=0;
void setup() {

    Serial.begin(115200);
    //https://github.com/espressif/arduino-esp32/issues/8080
    //https://community.platformio.org/t/esp32-c3-framework-arduino-serial-print-usb/30464
    Serial.setDebugOutput(true); // don't sent debug to UART pins
    
    for (int delaysec = 10; delaysec > 0; delaysec--)
    {
      delay(1000);
      ESP_LOGI("I2S-DMA", "Starting in %d...", delaysec);
    }



  // Display Setup
  dma_display = new MatrixPanel_I2S_DMA();
  dma_display->begin();
  //dma_display->clearScreen();

  dma_display->_generate_i2s_test_output();

  ESP_LOGI("I2S-DMA", "Free heap: %d", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  ESP_LOGI("I2S-DMA", "Free SPIRAM: %d", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));


  lastMillis = millis();
}

uint8_t wheelval = 0;
uint64_t frameCounts = 0;
void loop() {

     // animate by going through the colour wheel for the first two lines
//    drawText(wheelval);
//    wheelval +=1;
//    delay(1);
//    frameCounts++;

  if(millis()-lastMillis>=5000)
  {
      ESP_LOGI("I2S-DMA", "Free heap: %d", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
      ESP_LOGI("I2S-DMA", "Free SPIRAM: %d", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

        // log frame rate to serial
        Serial.print("fps: ");
        Serial.println(frameCounts);
        lastMillis = millis();
        frameCounts=0;
  }

  
}