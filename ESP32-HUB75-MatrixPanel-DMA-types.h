#ifndef ESP32_HUB75_MATRIXPANEL_DMA_TYPES_H
#define ESP32_HUB75_MATRIXPANEL_DMA_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include "color_convert.h"

//#define ADD_VSYNC_OUT
//#define ADD_HSYNC_OUT

typedef enum{
  ROTATE_0 = 0,
  ROTATE_90,
  ROTATE_180,
  ROTATE_270,
}rotate_t;

typedef enum{
  CLK_POZITIVE = 0, 
  CLK_NEGATIVE
}clk_phase_t;

enum{
  DOUBLE_BUFF_OFF = false, 
  DOUBLE_BUFF_ON = true
};


enum {
  SHIFTREG = 0, 
  MBI5124,
  ICN2038,
  ICN2038S = ICN2038,
  FM6124 = ICN2038S,
  FM6126A = ICN2038S,    
  ICN2053,
  FM6353 = ICN2053,
  FM6353B = ICN2053,
  };
enum clk_freq_t{HZ_5M=5000000, HZ_10M=10000000, HZ_13M=13000000, HZ_20M=20000000};

typedef struct{
  #ifdef ADD_VSYNC_OUT
  int8_t r1, g1, b1, r2, g2, b2, a, b, c, d, e, lat, oe, clk, vsync;
  #else
  int8_t r1, g1, b1, r2, g2, b2, a, b, c, d, e, lat, oe, clk;
  #endif
} hub75_pins_t;

typedef struct  HUB75_DMA_CFG_S{
  // physical width of a single matrix panel module (in pixels, usually it is 64 ;) )
  uint16_t mx_width;
  // physical height of a single matrix panel module (in pixels, usually amost always it is either 32 or 64)
  uint16_t mx_height;
  // number panels of width
  uint16_t mx_count_width;
  // number panels of height
  uint16_t mx_count_height;
    //GPIO pins mapping
  hub75_pins_t gpio;
  // Matrix driver chip type - default is a plain shift register
  uint8_t driver;
  // I2S clock speed
  clk_freq_t clk_freq;
  /**
   *  I2S clock phase
   *  0 (default) - data lines are clocked with negative edge
   *  Clk  /¯\_/¯\_/
   *  LAT  __/¯¯¯\__
   *  EO   ¯¯¯¯¯¯\___
   *
   *  1 - data lines are clocked with positive edge
   *  Clk  \_/¯\_/¯\
   *  LAT  __/¯¯¯\__
   *  EO   ¯¯¯¯¯¯\__
   *
   */
  clk_phase_t clk_phase;
  // How many clock cycles to blank OE before/after LAT signal change, default is 1 clock
  // use DMA double buffer (twice as much RAM required)
  // для Shift драйверов двойной буфер включатеся по double_buff или double_dma_buff 
  // для ICN2053 драйверов двойные буфера double_buff (первичный видео буфер) и double_dma_buff независимы (буфер вывода)
  //двойной DMA буфер(SHIFT) или двойной первичный видео буфер (ICN2053)
  color_depth_t color_depth;
  bool double_buff;
  //двойной DMA буфер
  bool double_dma_buff;
   //rotate_t rotate_coord;
   //bool serpentine_chain;
   //bool top_down_chain;
   //bool chain_top_down;
   //bool s_chain_party;
  //uint8_t latch_blanking;
  bool decoder_595;
}hub75_cfg_t; // end of structure HUB75_I2S_CFG

// Definitions below should NOT be ever changed without rewriting library logic
typedef uint16_t ESP32_I2S_DMA_STORAGE_TYPE;  // DMA output of one uint16_t at a time.
enum{SIZE_DMA_TYPE = sizeof(ESP32_I2S_DMA_STORAGE_TYPE)};
#define ESP32_I2S_DMA_MODE I2S_PARALLEL_WIDTH_16    // From esp32_i2s_parallel_v2.h = 16 bits in parallel

#endif
