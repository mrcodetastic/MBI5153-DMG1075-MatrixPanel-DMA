#ifndef _ESP32_RGB_64_32_MATRIX_PANEL_I2S_DMA
#define _ESP32_RGB_64_32_MATRIX_PANEL_I2S_DMA
/***************************************************************************************/
/* Core ESP32 hardware / idf includes!                                                 */
#include <vector>
#include <memory>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_attr.h>

// #include <Arduino.h>
#include "platforms/platform_detect.hpp"

#ifdef USE_GFX_ROOT
#include <FastLED.h>
#include "GFX.h" // Adafruit GFX core class -> https://github.com/mrfaptastic/GFX_Root
#elif !defined NO_GFX
#include "Adafruit_GFX.h" // Adafruit class with all the other stuff
#endif


#ifndef MATRIX_WIDTH
#define MATRIX_WIDTH  78 
#endif

#ifndef MATRIX_HEIGHT
#define MATRIX_HEIGHT 78 
#endif


// RGB Panel Scan Lines
#define PANEL_SCAN_LINES        20 // panel scan lines / rows
#define PANEL_MBI_LED_CHANS     16  // number of led channels per MBI IC
#define PANEL_MBI_CHAIN_LEN      5  // number of ic's changed for each subpixel

#define PANEL_PHY_RES_X 78
#define PANEL_PHY_RES_Y 78

#define PANEL_MBI_RES_X 80
#define PANEL_MBI_RES_Y 80


/***************************************************************************************/
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

#define ghost_elimination_ON    0b11   //послесвечение выключенно
#define ghost_elimination_OFF   0   //послесвечение включено
#define gray_scale_13           1   //шкала серого 13 бит
#define gray_scale_14           0   //шкала серого 14 бит

/* GCLK multiplier 
MBI5153 provides a GCLK multiplier function by setting the configuration register1 bit [6]. The default value is set 
to ’0’ for GCLK multiplier disable = 513 required for each row.

GCLK multiplier enabled (configuration register1 bit [6] = 1) 
= 257 gclocks for each row!
*/

//25600
// 10260 
#define gclk_multiplier_ON_clks  257   // GCLK Multipler On - You MUST use exactly 257 clocks for each rowscan!
#define gclk_multiplier_OFF_clks 513  // GCLK Multipler Off - You MUST use exactly 513 clocks for each rowscan!


#define gclk_multiplier_ON      1   // GCLK Multipler On - You MUST use exactly 257 clocks for each rowscan!
#define gclk_multiplier_OFF     0   // GCLK Multipler On - You MUST use exactly 513 clocks for each rowscan!
#define current_1               15  //ток на светодиоде
#define current_2               63  //ток на светодиоде
#define current_3               35  //ток на светодиоде
#define current_4               20  //LED current

#define clock_delay             0
#define pwm_hi                  65535  
#define pwm_lo                  0 


/***************************************************************************************/
#define ESP32_I2S_DMA_STORAGE_TYPE uint32_t

// For DMA GLCK and Address Line Data
#define BIT_GCLK  (1 << 0)
#define BIT_DCLK  (1 << 1)
#define BIT_LAT   (1 << 2)
#define BIT_A     (1 << 3)
#define BIT_B     (1 << 4)
#define BIT_C     (1 << 5)
#define BIT_D     (1 << 6)
#define BIT_E     (1 << 7)

// For DMA Greyscale and other data
#define BIT_COLOR_POS_START 8
#define BIT_G1   (1 << 8)
#define BIT_B1   (1 << 9)
#define BIT_R1   (1 << 10)
#define BIT_G2   (1 << 11)
#define BIT_B2   (1 << 12)
#define BIT_R2   (1 << 13)
#define BIT_G3   (1 << 14)
#define BIT_B3   (1 << 15)
#define BIT_R3   (1 << 16)
#define BIT_G4   (1 << 17)
#define BIT_B4   (1 << 18)
#define BIT_R4   (1 << 19)
#define BIT_RGB_ALL (BIT_G1 | BIT_B1 | BIT_R1 | BIT_G2 | BIT_B2 | BIT_R2 | BIT_G3 | BIT_B3 | BIT_R3 | BIT_G4 | BIT_B4 | BIT_R4 )

/***************************************************************************************/

/**
 * Get the number of descriptors required for a given buffer size.
 *
 * @param data_size Size to check descriptor num.
 *
 * @return Numbers required.
 */
// from esp32s2\include\soc\include\soc\lldesc.h
inline int lldesc_get_required_num_constrained(int data_size, int max_desc_size)
{
    return (data_size + max_desc_size - 1) / max_desc_size;
}

/***************************************************************************************/



// Contains all the shit sent out at once to the panel
struct ParallelDataOutputStruct
{
  uint32_t length     = 0; // length in parallel bits (not aggregate) 10 x uint32_t's sent in parallel = length of 10
  bool     in_psram   = false;
  ESP32_I2S_DMA_STORAGE_TYPE *data;

  // constructor - allocates DMA-capable memory to hold the struct data
  ParallelDataOutputStruct(size_t _length, bool _in_psram = false) : length(_length), in_psram(_in_psram) {

    // PSRAM - THIS WORKS
    // No longer have double buffer in the same struct - have a different struct
    if (in_psram == true) {
        ESP_LOGI("I2S-DMA", "Allocating %d bytes **PSRAM** for DMA parallel data output.", getByteSize());
        data = (ESP32_I2S_DMA_STORAGE_TYPE *)heap_caps_aligned_alloc(16, getByteSize(), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    } else {
        ESP_LOGI("I2S-DMA", "Allocating %d bytes **Internal Memeory** for DMA parallel data output.", getByteSize());
        data = (ESP32_I2S_DMA_STORAGE_TYPE *)heap_caps_malloc(getByteSize(), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    }
    if (data == nullptr)
    {
      ESP_LOGE("I2S-DMA", "CRITICAL ERROR: Unable to allocate DMA memory.\r\n");

      return;
      // TODO: should we release all previous rowBitStructs here???
    }

    // Fill with zeros to start with
    //memset(data, 0b11, getByteSize());    
    memset(data, 0b0, getByteSize());    

  }

  size_t getByteSize() {
    return length * sizeof(ESP32_I2S_DMA_STORAGE_TYPE);
  };

  size_t get_dma_lldesc_required() {
    return lldesc_get_required_num_constrained(getByteSize(), DMA_MAX);
  }

  // starts from 0 index
  uint32_t *get_dma_lldesc_data_ptr(int offset) {
    return &(data[offset*DMA_MAX]);

  }

  ~ParallelDataOutputStruct() { delete data; }  
};

/***************************************************************************************/

#ifndef NO_CIE1931

static const uint16_t DRAM_ATTR lumConvTab[] = {
    0, 27, 56, 84, 113, 141, 170, 198, 227, 255, 284, 312, 340, 369, 397, 426,
    454, 483, 511, 540, 568, 597, 626, 657, 688, 720, 754, 788, 824, 860, 898, 936,
    976, 1017, 1059, 1102, 1146, 1191, 1238, 1286, 1335, 1385, 1436, 1489, 1543, 1598, 1655, 1713,
    1772, 1833, 1895, 1958, 2023, 2089, 2156, 2225, 2296, 2368, 2441, 2516, 2592, 2670, 2750, 2831,
    2914, 2998, 3084, 3171, 3260, 3351, 3443, 3537, 3633, 3731, 3830, 3931, 4034, 4138, 4245, 4353,
    4463, 4574, 4688, 4803, 4921, 5040, 5161, 5284, 5409, 5536, 5665, 5796, 5929, 6064, 6201, 6340,
    6482, 6625, 6770, 6917, 7067, 7219, 7372, 7528, 7687, 7847, 8010, 8174, 8341, 8511, 8682, 8856,
    9032, 9211, 9392, 9575, 9761, 9949, 10139, 10332, 10527, 10725, 10925, 11127, 11332, 11540, 11750, 11963,
    12178, 12395, 12616, 12839, 13064, 13292, 13523, 13757, 13993, 14231, 14473, 14717, 14964, 15214, 15466, 15722,
    15980, 16240, 16504, 16771, 17040, 17312, 17587, 17865, 18146, 18430, 18717, 19006, 19299, 19595, 19894, 20195,
    20500, 20808, 21119, 21433, 21750, 22070, 22393, 22720, 23049, 23382, 23718, 24057, 24400, 24745, 25094, 25446,
    25802, 26160, 26522, 26888, 27256, 27628, 28004, 28382, 28765, 29150, 29539, 29932, 30328, 30727, 31130, 31536,
    31946, 32360, 32777, 33197, 33622, 34049, 34481, 34916, 35354, 35797, 36243, 36692, 37146, 37603, 38064, 38528,
    38996, 39469, 39945, 40424, 40908, 41395, 41886, 42382, 42881, 43383, 43890, 44401, 44916, 45434, 45957, 46484,
    47014, 47549, 48088, 48630, 49177, 49728, 50283, 50842, 51406, 51973, 52545, 53120, 53700, 54284, 54873, 55465,
    56062, 56663, 57269, 57878, 58492, 59111, 59733, 60360, 60992, 61627, 62268, 62912, 63561, 64215, 64873, 65535};
#endif


/***************************************************************************************/
#ifdef USE_GFX_ROOT
class MatrixPanel_I2S_DMA : public GFX
{
#elif !defined NO_GFX
class MatrixPanel_I2S_DMA : public Adafruit_GFX
{
#else
class MatrixPanel_I2S_DMA
{
#endif

  // ------- PUBLIC -------
public:
  /**
   * MatrixPanel_I2S_DMA
   *
   * default predefined values are used for matrix configuration
   *
   */
  MatrixPanel_I2S_DMA()
#ifdef USE_GFX_ROOT
      : GFX(MATRIX_WIDTH, MATRIX_HEIGHT)
#elif !defined NO_GFX
      : Adafruit_GFX(MATRIX_WIDTH, MATRIX_HEIGHT)
#endif
  {
  }


  /* Propagate the DMA pin configuration, allocate DMA buffs and start data output, initially blank */
  bool begin();

  // Obj destructor
  virtual ~MatrixPanel_I2S_DMA() {
    dma_bus.release();
  }

  // Adafruit's BASIC DRAW API (565 colour format)
  virtual void drawPixel(int16_t x, int16_t y, uint16_t color); // overwrite adafruit implementation

  void fillScreenRGB888(uint8_t r, uint8_t g, uint8_t b);
  void drawPixelRGB888(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b);

#ifdef USE_GFX_ROOT
  // 24bpp FASTLED CRGB colour struct support
  void fillScreen(CRGB color);
  void drawPixel(int16_t x, int16_t y, CRGB color);
#endif

  // Colour 444 is a 4 bit scale, so 0 to 15, colour 565 takes a 0-255 bit value, so scale up by 255/15 (i.e. 17)!
  static uint16_t color444(uint8_t r, uint8_t g, uint8_t b) { return color565(r * 17, g * 17, b * 17); }

  // Converts RGB888 to RGB565
  static uint16_t color565(uint8_t r, uint8_t g, uint8_t b); // This is what is used by Adafruit GFX!

  /**
   * @brief - convert RGB565 to RGB888
   * @param uint16_t colour - RGB565 input colour
   * @param uint8_t &r, &g, &b - refs to variables where converted colors would be emplaced
   */
  static void color565to888(const uint16_t color, uint8_t &r, uint8_t &g, uint8_t &b);


  /**
   * Stop the ESP32 DMA Engine. Screen will forever be black until next ESP reboot.
   */
  void stopDMAoutput() {
    // i2s_parallel_stop_dma(ESP32_I2S_DEVICE);
    dma_bus.dma_transfer_stop();
  }


  /***************************************************************************************/

  void _generate_i2s_test_output() { // buffer 1 only

    for (int i = 0; i < 512; i++) 
    {
      output_bits_greyscale->data[i] = i;  

      if ( (i%2) == 0) { output_bits_greyscale->data[i] |= BIT_R4; } // bit 19 / GPIO_NUM_8 - toggle last bit

    }

    for (int i = 1024; i < 2048; i++) 
    {
      output_bits_greyscale->data[i] = ~0;  

    }

  }

  void mbi_update_frame(bool configure_latches) {
      if (configure_latches)
      {
          int counter = 0;
          for (int row = 0; row < PANEL_SCAN_LINES; row++) {
            for (int chan = 0; chan < PANEL_MBI_LED_CHANS; chan++) {
              for (int ic = 0; ic < PANEL_MBI_CHAIN_LEN; ic++) { // number of chained ICs

                  //1600 iterations for one panel

                  // data latch on the last bit, when sending the last byte set latch=1
                  int latch = 0;
                  if (ic == 4) { latch = 1; }  // latch on last channel / ic

                  int bit_offset = 16;
                  while (bit_offset > 0)  // shift out MSB first per the documentation.
                  {
                    bit_offset--;  // start from 15

                    if (latch == 1 && bit_offset == 0) {
                      output_bits_greyscale->data[counter] |=  BIT_LAT;               
                    } 
                    counter++;             
                  }

              }
            }
          }      
      }

      //ESP_LOGD("I2S-DMA", "Sending greyscale data buffer out via LCD DMA.");
      //dma_bus.send_stuff_once(dma_grey_gpio_data, dma_grey_buffer_size, true); // sending payload hence TRUE 
    
  } // mbi_update_frame

  /*
    Gray Scale Mode and Scan-type S-PWM
    MBI5153 provides a selectable 14-bit or 13-bit gray scale by setting the configuration register1 bit [7]. The default 
    value is set to ’0’ for 14-bit color depth. In 14-bit gray scale mode, users should still send 16-bit data with 2-bit ‘0’ in 
    LSB bits. For example, {14’h1234, 2’h0}. 
    MBI5153 has a smart S-PWM technology for scan type. With S-PWM, the total PWM cycles can be broken into MSB 
    (Most Significant Bits) and LSB (Least Significant Bits) of gray scale cycles. The MSB information can be broken 
    down into many refresh cycles to achieve overall same high bit resolution.

    The 16384 GCLKs (14-bit) PWM cycle of MBI5052/53 is divided into 32 sections, each section has 512 GCLKs.
  */
  void mbi_set_pixel(uint8_t x, uint8_t  y, uint8_t r_data, uint8_t g_data, uint8_t b_data) {

    if ( x >= PANEL_PHY_RES_X || y >= PANEL_PHY_RES_Y) {
      return;
    }

    x += 2; // offset for missing pixels on the left


    //14-bit resolution = 16,384
    // 8-bit resolutoin = 255
    // ... that's 64 times larger
    uint16_t g_14bit_data = (g_data*64) << 2;  
    uint16_t b_14bit_data = (b_data*64) << 2;    
    uint16_t r_14bit_data = (r_data*64) << 2; // could just bit shift by << 6  in total instead


    ESP_LOGD("I2S-DMA", "Converted 14bit colour value r: %d,  g: %d,  b: %d", g_14bit_data, g_14bit_data, b_14bit_data);  


    // x and y positions start from 0, so 0-78 are valid values only
    uint16_t g_gpio_bitmask = BIT_G1; // bit 0
    uint16_t b_gpio_bitmask = BIT_B1; // bit 1
    uint16_t r_gpio_bitmask = BIT_R1; // bit 2

    uint16_t _colourbitoffset = BIT_COLOR_POS_START + (y/PANEL_SCAN_LINES) * 3; // three is an important bit
    uint16_t _colourbitclear  = ~(0b111 << _colourbitoffset); // invert 3 bits

    g_gpio_bitmask = g_gpio_bitmask << _colourbitoffset;
    b_gpio_bitmask = b_gpio_bitmask << _colourbitoffset;
    r_gpio_bitmask = r_gpio_bitmask << _colourbitoffset;  

    // Row offset + channel offset + individual IC LED offset
    int y_normalised  = y % PANEL_SCAN_LINES; // Only have 20 rows of data... 
    int bit_start_pos = (1280*y_normalised)+((x%16)*80)+((x/16)*16);

    int bit_offset = 16;
    while (bit_offset > 0)  // shift out MSB first per the documentation.
    {
      bit_offset--;  // start from 15

      uint16_t mask = 1 << bit_offset;

      output_bits_greyscale->data[bit_start_pos] &= _colourbitclear; // clear relevant rgb bits

      if ( g_14bit_data & mask ) { 
        output_bits_greyscale->data[bit_start_pos] |=  g_gpio_bitmask;
      }

      if ( b_14bit_data & mask ) { 
        output_bits_greyscale->data[bit_start_pos] |=  b_gpio_bitmask;
      }

      if ( r_14bit_data & mask ) { 
        output_bits_greyscale->data[bit_start_pos] |=  r_gpio_bitmask;
      }      

      ESP_LOGD("I2S-DMA", "Setting output_bits_greyscale from bit_start_pos %d. Value %d", bit_start_pos, output_bits_greyscale->data[bit_start_pos]);  

      bit_start_pos++;
    }

  } // mbi_greyscale_data_set_pixel


  /* Generate an arbitaray binary output with the DCLK created in code */
  void _set_command_output(int latch_dclks) {

    // *  DCLK  /¯\_/¯\_/
    // *  LAT   __/¯¯¯\__      
    for (int i = 0; i < output_bits_command->length; i++) output_bits_command->data[i] = 0;

    for (int i = 0; i < 64; i++) { // no command is longer than 32 cycles of DCLK

     if (i % 2 != 0) {      
          output_bits_command->data[i] = BIT_DCLK; // pulse high every second latch
     } /*else {
          output_bits_command.data[i] = 0b0; 
     }   */   
    }

    for (int i = 0; i < latch_dclks*2; i++) { // latch state is read on transition to DCLK high only
      output_bits_command->data[i] |= BIT_LAT; // set the latch bit to high from the start
    }  
  } // end _set_command_output


    /*pre-activation command - sent before sending configuration register data*/
    void mbi_pre_active_dma() {

      ESP_LOGD("I2S-DMA", "Send MBI Pre-Active.");
      _set_command_output(14); 

      // Note: Make sure latch stays low or it'll get out of pre-active mode before register data is sent!
      // "In the durations of [between] Pre-Active command and Write Configuration command, LE must pull to low to prevent leaving 
      // the Pre-Active mode and the configuration data becomes invalid."

      // Should be at least two DCLKS before any subsequent
    }

    /*vertical sync - updates frame data on outputs, used in conjunction with vertical scan*/
    // 9. GCLK must keep at low level more than 7ns before MBI5152 receives the Vsync signa
    // 7ns = 142 Mhz clock speed...
    /* After the last data latch command, it needs at least 50 GCLKs to read the gray scale data into internal display 
    buffer before the Vsync command comes. And display is updated immediately until MBI5051/52/53 receives 
    the Vsync signal (high pulse of LE pin is sampled by 3-DCLK rising edges), as figure 6 shows. 
    */
    void mbi_v_sync_dma() {
      ESP_LOGD("I2S-DMA", "Send MBI Vert Sync.");
      _set_command_output(2);       

      /*
        gpio_set_level((gpio_num_t)MBI_LAT, 1);
        mbi_clock(2);
        gpio_set_level((gpio_num_t)MBI_LAT, 0);
      */  
    }

    /*soft reset*/
    void mbi_soft_reset_dma() {
      /* “Software reset” command makes MBI5153 go back to the initial state except configuration register value. After this 
      command is received, the output channels will be turned off and will display again with last gray-scale value after 
      new “Vsync” command is received.
      */
      ESP_LOGD("I2S-DMA", "Send MBI Soft Reset.");
      _set_command_output(10);        
    }


    /*отправка регистра конфигурации*/
    /* Need to set config for all MBI input on the panel at once as they all share the same latch pin */
    void _set_config_output(int &bit_start_pos, uint16_t config_reg_value, bool send_latch, bool config_reg2) {
      //  ESP_LOGD("I2S-DMA", "Send MBI config");

      // Number of DCLK Rising Edge when LE is asserted 
      // Write Configuration 1 = 4
      // Write Configuration 2 = 8
      int latch_trigger_point = (config_reg2) ? 8:4; 

      for (int bit = 15; bit >= 0; bit--) {
        
          int bitval = ((config_reg_value >> bit) & 1 );

          ESP32_I2S_DMA_STORAGE_TYPE mbi_rgb_sdi_val = 0;
          if (bitval) {
              mbi_rgb_sdi_val = BIT_RGB_ALL; // all <BI colour channels get the same config for now.
          }

          if ( (bit < latch_trigger_point) && (send_latch == true) )  { // for reg1, data latch on the last 4 bits when latch is true (on last send)

              //std::cout << "Latch." << std::endl;   
              mbi_rgb_sdi_val |= BIT_LAT;
          }     
          
          //
          output_bits_command->data[bit_start_pos++] =  mbi_rgb_sdi_val;
          output_bits_command->data[bit_start_pos++] =  mbi_rgb_sdi_val | BIT_DCLK;

      } // iterate through bits

    } // mbi_send_config_dma

    void mbi_send_config_reg1_dma() {

      // Step 1) Send configuration for 
      uint16_t config_reg1_val = 0;

      int ghost_elimination    = ghost_elimination_ON;
      int line_num             = PANEL_SCAN_LINES-1;
      int gray_scale           = gray_scale_14; 
      int gclk_multiplier      = gclk_multiplier_OFF;
      int current              = current_1; // change as required by channel

      //Documentation says set bits E and F of Config1 Reg to 1
      config_reg1_val = (config_reg1_val | (ghost_elimination << 14) | (line_num << 8) | (gray_scale << 7) | (gclk_multiplier << 6) | (current));

      int bit_start_pos = 0;
      for (int i = 0; i < PANEL_MBI_CHAIN_LEN; i++) {
        _set_config_output(bit_start_pos, config_reg1_val, (i == (PANEL_MBI_CHAIN_LEN - 1)), false);  // on last mbi chip, latch  
      }

      //dma_bus.send_stuff_once(dma_grey_gpio_data, dma_output_pos*sizeof(ESP32_GREY_DMA_STORAGE_TYPE), false);

    }

    /*настройка регистра конфигурации*/
    void mbi_send_config_reg2_dma() {
      // F E D C B A 9 8 7 6 5 4 3 2 1 0
      //uint16_t config_reg2_val = 0b0001000000010000; // default value apparently

      uint16_t config_reg2_val = 0b1001000000011110; // removes ghosting except for red
      int bit_start_pos = 0;

      for (int i = 0; i < PANEL_MBI_CHAIN_LEN; i++) {
        _set_config_output(bit_start_pos, config_reg2_val, (i == (PANEL_MBI_CHAIN_LEN - 1)), true);  // on last mbi chip, latch  
      }

      //dma_bus.send_stuff_once(dma_grey_gpio_data, dma_output_pos*sizeof(ESP32_GREY_DMA_STORAGE_TYPE), false);

    }





  

  // ------- PROTECTED -------
protected:

  Bus_Parallel16 dma_bus;

  /* Update a specific pixel in the DMA buffer to a colour */
  void updateMatrixDMABuffer(uint16_t x, uint16_t y, uint8_t red, uint8_t green, uint8_t blue);

private:

  /* Pixel data is organized from LSB to MSB sequentially by row, from row 0 to row matrixHeight/matrixRowsInParallel
   * (two rows of pixels are refreshed in parallel)
   * Memory is allocated (malloc'd) by the row, and not in one massive chunk, for flexibility.
   * The whole DMA framebuffer is just a vector of pointers to structs with ESP32_I2S_DMA_STORAGE_TYPE arrays
   * Since it's dimensions is unknown prior to class initialization, we just declare it here as empty struct and will do all allocations later.
   * Refer to rowBitStruct to get the idea of it's internal structure
   */
  ParallelDataOutputStruct* output_bits_greyscale;
  ParallelDataOutputStruct* output_bits_gclk_only;
  ParallelDataOutputStruct* output_bits_command;  

}; // end Class header

/***************************************************************************************/
// https://stackoverflow.com/questions/5057021/why-are-c-inline-functions-in-the-header
/* 2. functions declared in the header must be marked inline because otherwise, every translation unit which includes the header will contain a definition of the function, and the linker will complain about multiple definitions (a violation of the One Definition Rule). The inline keyword suppresses this, allowing multiple translation units to contain (identical) definitions. */

/**
 * @brief - convert RGB565 to RGB888
 * @param uint16_t colour - RGB565 input colour
 * @param uint8_t &r, &g, &b - refs to variables where converted colours would be emplaced
 */
inline void MatrixPanel_I2S_DMA::color565to888(const uint16_t color, uint8_t &r, uint8_t &g, uint8_t &b)
{
  r = (color >> 8) & 0xf8;
  g = (color >> 3) & 0xfc;
  b = (color << 3);
  r |= r >> 5;
  g |= g >> 6;
  b |= b >> 5;
}

inline void MatrixPanel_I2S_DMA::drawPixel(int16_t x, int16_t y, uint16_t color) // adafruit virtual void override
{
  uint8_t r, g, b;
  color565to888(color, r, g, b);
  updateMatrixDMABuffer(x, y, r, g, b);
}

inline void MatrixPanel_I2S_DMA::drawPixelRGB888(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b)
{
  updateMatrixDMABuffer(x, y, r, g, b);
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed colour
// https://github.com/squix78/ILI9341Buffer/blob/master/ILI9341_SPI.cpp
inline uint16_t MatrixPanel_I2S_DMA::color565(uint8_t r, uint8_t g, uint8_t b)
{
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#endif
