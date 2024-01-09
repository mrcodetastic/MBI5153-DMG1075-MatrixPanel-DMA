# ESP32S3-MBI5153-DMG1075-LED-MatrixPanel
An experimental Arduino sketch to drive an 'DMG1075' 78x78px RGB LED Matrix Panel (based on the MBI5153).

Based on ESP32S3 only.

### Still to do
* Implement DMA, PWM or RMT based driving based on [advice](https://github.com/mrfaptastic/ESP32-HUB75-MatrixPanel-DMA/discussions/324#discussioncomment-7845435) (done)
* Hook it up to a general graphic library, and local 24bit pp buffer.

### How to use
* Generate an 80x80 pixel image and convert to c array using CF_TRUE_COLOR at https://lvgl.io/tools/imageconverter
* Add this to the test_pattern.h file

Note: Left 2 and Bottom 2 rows will be dropped from the image data as the pysical panel is only 78x78 pixels (however we need to feed to panel 80x80px worth of data)

### Further Information
Further details on the panel here: https://led.limehouselabs.org/docs/tiles/dmg1083/

![Front](docs/front.jpg)

![Back](docs/back.jpg)
