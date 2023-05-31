/*
 * Portions of this code are adapted from Aurora: https://github.com/pixelmatix/aurora
 * Copyright (c) 2014 Jason Coon
 *
 * Portions of this code are adapted from LedEffects Plasma by Robert Atkins: https://bitbucket.org/ratkins/ledeffects/src/26ed3c51912af6fac5f1304629c7b4ab7ac8ca4b/Plasma.cpp?at=default
 * Copyright (c) 2013 Robert Atkins
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
 
#include "ESP32-HUB75-MatrixPanel-DMA.h"
#include <FastLED.h>

// placeholder for the matrix object
MatrixPanel_DMA *dma_display = nullptr;


uint16_t time_counter = 0, cycles = 0, fps = 0;
unsigned long fps_timer;

CRGB currentColor;
CRGBPalette16 palettes[] = {HeatColors_p, LavaColors_p, RainbowColors_p, RainbowStripeColors_p, CloudColors_p};
CRGBPalette16 currentPalette = palettes[0];


CRGB ColorFromCurrentPalette(uint8_t index = 0, uint8_t brightness = 255, TBlendType blendType = LINEARBLEND) {
  return ColorFromPalette(currentPalette, index, brightness, blendType);
}

void setup() {
  
  Serial.begin(115200);
  
  Serial.println(F("*****************************************************"));
  Serial.println(F("*        ESP32-HUB75-MatrixPanel-DMA DEMO           *"));
  Serial.println(F("*****************************************************"));

  #define PANEL_WIDTH 80
  #define PANEL_HEIGHT 40
  #define PANEL_WIDTH_CNT 3
  #define PANEL_HEIGHT_CNT 3
  #define PANE_WIDTH PANEL_WIDTH*PANEL_WIDTH_CNT
  #define PANE_HEIGHT PANEL_HEIGHT*PANEL_HEIGHT_CNT

  hub75_cfg_t mxconfig = {
    .mx_width = PANEL_WIDTH,
    .mx_height = PANEL_HEIGHT,
    .mx_count_width = PANEL_WIDTH_CNT,
    .mx_count_height = PANEL_HEIGHT_CNT,
    .gpio = {
    .r1 = R1_PIN,
    .g1 = G1_PIN,
    .b1 = B1_PIN,
    .r2 = R2_PIN,
    .g2 = G2_PIN,
    .b2 = B2_PIN,
    .a = A_PIN,
    .b = B_PIN,
    .c = C_PIN,
    .d = D_PIN,
    .e = E_PIN,
    .lat = LAT_PIN,
    .oe = OE_PIN,
    .clk = CLK_PIN,
    },
    .driver = ICN2053,
    .clk_freq = HZ_13M, 
    .clk_phase = CLK_POZITIVE,
    .color_depth = COLORx16,//PIXEL_COLOR_DEPTH
    .double_buff = DOUBLE_BUFF_ON,
    .double_dma_buff = DOUBLE_BUFF_ON,
    .decoder_INT595 = false,
  };

  // OK, now we can create our matrix object
  // if mx_count_height > 1 - auto use 
  // virtual panel uses additional parameter:
  // VIRTUAL_S_TOP_DOWN - serpantine from top down (default, may be skipped)
  // VIRTUAL_S_BOTTOM_UP - serpantine from bottom up
  // VIRTUAL_TOP_DOWN - unidirectional from top down
  // VIRTUAL_BOTTOM_UP - unidirectional from bottom up
  //dma_display = new MatrixPanel_DMA(mxconfig, VIRTUAL_S_TOP_DOWN);
  dma_display = new MatrixPanel_DMA(mxconfig);


  // Allocate memory and start DMA display
  if( not dma_display->begin() )
      Serial.println("****** !KABOOM! I2S memory allocation failed ***********");

  // let's adjust default brightness to about 75%
  dma_display->setBrightness8(192);    // range is 0-255, 0 - 0%, 255 - 100%
  //set rotate for new drawing
  //dma_display->setRotate(ROTATE_90); //ROTATE_0, ROTATE_90, ROTATE_180, ROTATE_270
  //set mirror for new drawing
  //dma_display->setMirrorX(true);
  //dma_display->setMirrorY(true);
 
  // well, hope we are OK, let's draw some colors first :)
  Serial.println("Fill screen: RED");
  dma_display->fillScreenRGB888(255, 0, 0);
  dma_display->flipDMABuffer();
  delay(1000);

  Serial.println("Fill screen: GREEN");
  dma_display->fillScreenRGB888(0, 255, 0);
  dma_display->flipDMABuffer();
  delay(1000);

  Serial.println("Fill screen: BLUE");
  dma_display->fillScreenRGB888(0, 0, 255);
  dma_display->flipDMABuffer();
  delay(1000);

  Serial.println("Fill screen: Neutral White");
  dma_display->fillScreenRGB888(64, 64, 64);
  dma_display->flipDMABuffer();
  delay(1000);

  Serial.println("Fill screen: black");
  dma_display->fillScreenRGB888(0, 0, 0);
  dma_display->flipDMABuffer();
  delay(1000);


  // Set current FastLED palette
  currentPalette = RainbowColors_p;
  Serial.println("Starting plasma effect...");
  fps_timer = millis();
}

void loop() {
  
    for (int x = 0; x < PANE_WIDTH; x++) {
            for (int y = 0; y <  PANE_HEIGHT; y++) {
                int16_t v = 0;
                uint8_t wibble = sin8(time_counter);
                v += sin16(x * wibble * 3 + time_counter);
                v += cos16(y * (128 - wibble)  + time_counter);
                v += sin16(y * x * cos8(-time_counter) / 8);

                currentColor = ColorFromPalette(currentPalette, (v >> 8) + 127); //, brightness, currentBlendType);
                dma_display->drawPixelRGB888(x, y, currentColor.r, currentColor.g, currentColor.b);
            }
    }
    dma_display->flipDMABuffer();

    ++time_counter;
    ++cycles;
    ++fps;

    if (cycles >= 1024) {
        time_counter = 0;
        cycles = 0;
        currentPalette = palettes[random(0,sizeof(palettes)/sizeof(palettes[0]))];
    }

    // print FPS rate every 5 seconds
    // Note: this is NOT a matrix refresh rate, it's the number of data frames being drawn to the DMA buffer per second
    if (fps_timer + 5000 < millis()){
      Serial.printf_P(PSTR("Effect fps: %d\n"), fps/5);
      fps_timer = millis();
      fps = 0;
    }
} // end loop
