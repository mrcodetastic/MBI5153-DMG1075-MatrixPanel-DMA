#include <Matrix.h>
#include <FastNoise.h>
// #include "e131.h"
// #include "wificonnect.h"


/*
  // MBI5152 Application Note V1.00- EN

  Section 2: The Setting of Gray Scale
  The setting of gray scale data describes as below.
  1. The sequence of input data starts from scan line 1 scan line 2….  scan line M-1 scan line M
  (M≦16)
  2. The data sequence of cascaded IC is ICnICn-1…. IC2IC1.
  3. The data sequence of each channel is ch15ch14…. ch0.
  4. The data length of each channel is 16-bits, and the default PWM mode is 16-bits. The sequence of gray
  sacle is bit15bit14bit13…bit0 as figure 4 shows. The 14-bits gray scale can be set through Bit[7]=1
  in configuration register 1, and the sequence of gray scale data is bit13bit12… bit0 0 0, the
  last 2-bits (LSB) are set to “0”.
  5. The frequency of GCLK must be higher than 20% of DCLK to get the correct gray scale data.
  6. LE executes the data latch to send gray scale data into SRAM. Each 16xN bits data needs a “data latch
  command”, where N means the number of cascaded driver.
  7. After the last data latch, it needs at least 50 GCLKs to read the gray scale data into internal display buffer
  before the Vsync command comes.
  8. Display is updated immediately when MBI5152 receives the Vsync signal.
  9. GCLK must keep at low level more than 7ns before MBI5152 receives the Vsync signal.
  10. The period of dead time (ie. The 1025th GCLK) must be larger than 100ns.

  The gray scale data needs the GCLK to save the data into SRAM. The frequency of GCLK must be higher
  than 20% of DCLK to get the correct data.

  // MBI5153
  After the last data latch command, it needs at least 50 GCLKs to read the gray scale data into internal display
  buffer before the Vsync command comes. And display is updated immediately until MBI5051/52/53 receives
  the Vsync signal (high pulse of LE pin is sampled by 3-DCLK rising edges), as figure 6 shows.

*/
static const char *TAG = "app_main";

Matrix matrix;

FastNoiseLite noise;
// E131 e131;

float simplexColorR = 0;
float simplexColorG = 100;
float simplexColorB = 150;

float simplexFrequency = 0.05;
float simplexSpeed = .002;

TaskHandle_t updateMatrixTaskHandle;
TaskHandle_t updateNoiseTaskHandle;
TaskHandle_t updateRegisterTaskHandle;

SemaphoreHandle_t matrixUpdatedSemaphore;


/************************************************************************/
float tx, nx, p;
float ty, ny, py;
float rot, rotx, roty, rotz, rotxx, rotyy, rotzz, rotxxx, rotyyy, rotzzz;
int i;          // 0 to 360
int fl, scale;  // focal length
int wireframe[12][2];

int originx = 40;
int originy = 40;  // 32

int front_depth = 20;
int back_depth = -20;

const char* Matrix::TAG = "Matrix";// Store cube vertices

int cube_vertex[8][3] = {
    {-20, -20, front_depth},
    {20, -20, front_depth},
    {20, 20, front_depth},
    {-20, 20, front_depth},
    {-20, -20, back_depth},
    {20, -20, back_depth},
    {20, 20, back_depth},
    {-20, 20, back_depth}};

/********************************************************************/

/********************************************************************/

void updateNoise() {
  // -------Fastnoise---------//
  // float col = .1;
  for (float i = 0; i < 78; i++) {
    for (float j = 0; j < 78; j++) {
      float col = (1 + noise.GetNoise(j, i, (millis() * simplexSpeed)))/2;

      matrix.drawPixel(j, i, (col * simplexColorR), (col * simplexColorG), (col * simplexColorB));
    }
  }
}

void timer_callback(void* arg) {
  updateNoise();
  matrix.update();
}

void setup_periodic_timer() {
  const esp_timer_create_args_t periodic_timer_args = {
      .callback = &timer_callback,
      .name = "periodic_60ms_timer"
  };

  esp_timer_handle_t periodic_timer;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 60000)); // 60,000 microseconds = 60ms
}

void testingTask(void *parameter) {
  const TickType_t xFrequency = pdMS_TO_TICKS(60); // 2000 ms interval (2 seconds)
  TickType_t xLastWakeTime = xTaskGetTickCount(); // Get the current tick count

  for (;;) {
    matrix.update();
    updateNoise(); // Call the testing function
    vTaskDelayUntil(&xLastWakeTime, xFrequency); // Delay until the next 2 seconds
  
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();

  //ESP_LOGE(TAG, "Time since last loop: %lu ms", currentTime - lastTime);
  lastTime = currentTime;

  }
}

void updateNoiseTask(void *parameter) {
  for (;;) {
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGD(TAG, "updateNoise %d", uxHighWaterMark);
    xSemaphoreTake(matrixUpdatedSemaphore, portMAX_DELAY); // Wait for updateMatrix to complete
    
    updateNoise();

  // ESP_LOGE(TAG, "Current timecode: %lu", millis());
  }
}

void updateMatrixTask(void *parameter) {
  const TickType_t xFrequency = pdMS_TO_TICKS(60); // 30 ms interval
  TickType_t xLastWakeTime = xTaskGetTickCount(); // Get the current tick count

  for (;;) {
    // Wait for the next cycle.
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGD(TAG, "updateMatrix: %d", uxHighWaterMark);
    matrix.update();
    xSemaphoreGive(matrixUpdatedSemaphore); // Signal that updateMatrix is done
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void updateRegisterTask(void *parameter) {
  const TickType_t xFrequency = pdMS_TO_TICKS(10000); // 30 ms interval
  TickType_t xLastWakeTime = xTaskGetTickCount(); // Get the current tick count

  for (;;) {
    // Wait for the next cycle.
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGD(TAG, "updateRegister: %d", uxHighWaterMark);
    matrix.updateRegisters();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


// Start the App
void setup(void) {
  // Serial.begin(115200);
  delay(2);
  Serial.println("Starting....");
  Serial.print("setup() running on core ");
  Serial.println(xPortGetCoreID());
  esp_task_wdt_deinit();

  // matrix = new Matrix();

  // connectWiFi();
  // e131.begin();

  matrix.initMatrix();

  noise.SetNoiseType(FastNoiseLite::NoiseType_OpenSimplex2S);
  noise.SetFrequency(simplexFrequency);
  matrixUpdatedSemaphore = xSemaphoreCreateBinary();

}


extern volatile int transfer_count;

void loop() {

  updateNoise();
  matrix.fillCircleDMA(40, 40, 5, 255, 0, 0);
  matrix.update();
  
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();

  if ((currentTime - lastTime) > 1000)
  {
    // Every Second
  //  log_e("Time since last loop: %lu ms", currentTime - lastTime);

    // Serial logging pauses stuff and causes flicker
    //log_e("Transfer count: %d", transfer_count);

    lastTime = currentTime;

  }




  // -------end---------//

  // -------e131---------//
  //     memset(dma_grey_gpio_data, 0, dma_grey_buffer_size);

  //   for (int i = 0; i < 78; i++) {
  //    for (int j = 0; j < 78; j++) {
  //       uint8_t col = wsRawData[(i*78+j)];
  //       mbi_set_pixel(j, i, col, col, col);
  //    }
  //  }

  //     mbi_update_frame(true);
  //     spi_transfer_loop_stop();
  //     mbi_v_sync_dma();
  //     spi_transfer_loop_restart();

  // ---------end--------//
}
