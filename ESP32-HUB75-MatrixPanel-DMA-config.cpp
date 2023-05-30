#include "ESP32-HUB75-MatrixPanel-DMA-config.h"

const hub75_cfg_t hub75_cfg_default= {
  .mx_width = PANEL_WIDTH,
  .mx_height = PANEL_HEIGHT,
  .mx_count_width = PANEL_COUNT_X,
  .mx_count_height = PANEL_COUNT_Y,
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
  #ifdef ADD_VSYNC_OUT
  .vsync = VSYNC_PIN,
  #endif
  #ifdef ADD_VSYNC_OUT
  .hsync = HSYNC_PIN,
  #endif
  },
  .driver = SHIFT_DRIVER,
  .clk_freq = CLK_CLOCK, 
  .clk_phase = CLK_PHASE,
  .color_depth = PIXEL_COLOR_DEPTH, 
  .double_buff = DOUBLE_BUFF,
  .double_dma_buff = DOUBLE_DMA_BUFF,
  .decoder_595 = DECODER_595,
  //  .latch_blanking = LAT_BLANKING,
};
