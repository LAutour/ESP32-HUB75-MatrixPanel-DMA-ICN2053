#include <Arduino.h>
#include "ESP32-HUB75-MatrixPanel-DMA.h"
#include "ESP32-HUB75-MatrixPanel-DMA-leddrivers.h"
//отрисовка без привязки к реализации

#ifndef _swap_int16
#define _swap_int16(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

#ifndef ABS
#define ABS(a)  (a >= 0)?(a):(-(a))  
#endif

#ifndef MIN
#define MIN(a,b) ((a) < (b))?(a):(b)  
#endif

#ifndef MAX
#define MIN(a,b) ((a) > (b))?(a):(b)  
#endif


//вынесенные процедуры отрисовки

void MatrixPanel_DMA::setColor(uint8_t r, uint8_t g, uint8_t b)
{
  if ((CurRGB.r == r)&&(CurRGB.g == g)&&(CurRGB.b == b)) return;
	CurRGB.r = r;
	CurRGB.g = g;
	CurRGB.b = b; 	
  CurColor = color565(r, g, b);    
}

void MatrixPanel_DMA::setColor(uint16_t color)
{
  if (color == CurColor) return;
  CurColor = color;
  color565to888(color, CurRGB.r, CurRGB.g, CurRGB.b);
}

void MatrixPanel_DMA::setTextColorRGB(uint8_t r, uint8_t g, uint8_t b)
{  
  setTextColor(color565(r,g,b));
}

void MatrixPanel_DMA::clearScreen()
{
  //Serial.println("clear");
  clearBuffer(0);
  if (m_cfg.double_buff)
  {
    clearBuffer(1);
  }
}

void MatrixPanel_DMA::drawPixel(int16_t x, int16_t y, uint16_t color) // adafruit virtual void override
{
  setColor(color);
  fillRect(x,y, 1, 1);
} 

void MatrixPanel_DMA::drawPixelRGB888(int16_t x, int16_t y, uint8_t r, uint8_t g,uint8_t b) 
{
  setColor(r, g, b);
  fillRect(x,y, 1, 1);
}

#ifdef USE_GFX_ROOT
// Support for CRGB values provided via FastLED
void MatrixPanel_DMA::drawPixel(int16_t x, int16_t y, CRGB color) 
{
  drawPixel( x, y, color.red, color.green, color.blue);
}

void MatrixPanel_DMA::fillScreen(CRGB color) 
{
  fillScreen(color565(color.red, color.green, color.blue));
}
#endif

void MatrixPanel_DMA::fillScreen(uint16_t color)  // adafruit virtual void override
{  
  //Serial.println("Fill");
  setColor(color);
  //Serial.println(back_buffer_id);
  if (color == 0) 
    clearBuffer(1);
  else 
    MatrixPanel_DMA::fillRectBuffer(0,0,pixels_per_row, m_cfg.mx_height); // RGB only (no pixel coordinate) version of 'updateMatrixDMABuffer'
} 

void MatrixPanel_DMA::fillScreenRGB888(uint8_t r, uint8_t g,uint8_t b)
{
  setColor(r, g ,b);
  MatrixPanel_DMA::fillRectBuffer(0,0,pixels_per_row, m_cfg.mx_height); // RGB only (no pixel coordinate) version of 'updateMatrixDMABuffer'
} 

void MatrixPanel_DMA::fillRect(int16_t x, int16_t y, int16_t w, int16_t h)
{
  fillRectBufferVirtual(x,y,w,h);
}

#ifndef NO_FAST_FUNCTIONS

void MatrixPanel_DMA::drawFastVLine(int16_t x, int16_t y, int16_t h)
{
  fillRect(x, y, 1, h);
}

void MatrixPanel_DMA::drawFastVLine(int16_t x, int16_t y, int16_t h, uint8_t r, uint8_t g, uint8_t b)
{ 
  setColor(r, g, b);
  fillRect(x, y, 1, h); 
};
void MatrixPanel_DMA::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
  setColor(color);
  fillRect(x, y, 1, h);
}

void MatrixPanel_DMA::drawFastHLine(int16_t x, int16_t y, int16_t w)
{
  fillRect(x, y, w, 1);
}

void MatrixPanel_DMA::drawFastHLine(int16_t x, int16_t y, int16_t w, uint8_t r, uint8_t g, uint8_t b)
{ 
  setColor(r, g, b);
  fillRect(x, y, w, 1);
};
void MatrixPanel_DMA::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
  setColor(color);
  fillRect(x, y, w, 1);
}

void MatrixPanel_DMA::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t r, uint8_t g, uint8_t b)
{
  setColor(r, g, b);
  fillRect(x, y, w, h);
}

void MatrixPanel_DMA::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  setColor(color);
  fillRect(x, y, w, h);
}


void MatrixPanel_DMA::drawRect(int16_t x, int16_t y, int16_t w, int16_t h)
{
  if ((w > 0)&&(h > 0))
  {
    drawFastHLine(x, y, w);
    drawFastHLine(x, y+h-1, w);
    drawFastVLine(x, y, h);
    drawFastVLine(x+w-1, y, h);
  }
}

void MatrixPanel_DMA::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t r, uint8_t g, uint8_t b)
{
  setColor(r, g, b);
  drawRect(x, y, w, h);
}

void MatrixPanel_DMA::drawRect(int16_t x, int16_t y, int16_t w, int16_t h,uint16_t color)
{
  setColor(color);
  drawRect(x, y, w, h);
}
#endif

void MatrixPanel_DMA::drawIcon (int* ico, int16_t x, int16_t y, int16_t cols, int16_t rows) 
{
/*  drawIcon draws a C style bitmap.  
//  Example 10x5px bitmap of a yellow sun 
//
  int half_sun [50] = {
      0x0000, 0x0000, 0x0000, 0xffe0, 0x0000, 0x0000, 0xffe0, 0x0000, 0x0000, 0x0000,
      0x0000, 0xffe0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xffe0, 0x0000,
      0x0000, 0x0000, 0x0000, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0x0000, 0x0000, 0x0000,
      0xffe0, 0x0000, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0x0000, 0xffe0,
      0x0000, 0x0000, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0x0000, 0x0000,
  };
  
  MatrixPanel_DMA matrix;

  matrix.drawIcon (half_sun, 0,0,10,5);
*/

  int i, j, offset;
  for (i = 0; i < rows; i++) 
  {
    for (j = 0; j < cols; j++) 
    {
      drawPixel(x + j, y + i, (uint16_t) ico[offset]);
      offset++;
    }
  }  
}

void MatrixPanel_DMA::_steepDrawPixelRGB(bool steep, int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b)
{
  if (steep) drawPixelRGB888(y,x,r,g,b);
  else drawPixelRGB888(x,y,r,g,b);
}

//Wu's line algorithm, fix point version
void MatrixPanel_DMA::_writeLineAARGB(bool steep, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t r, uint8_t g, uint8_t b) 
{
  int16_t w,h;
  if (steep) {w = _height; h = _width;}
  else {w = _width; h = _height;};

  _steepDrawPixelRGB(steep, x0, y0, r, g, b);
  _steepDrawPixelRGB(steep, x1, y1, r, g, b);
  
  //dy <<= 16;
  int32_t gradient = ((y1-y0)<< 16)/(x1-x0);
  int32_t y = (y0 << 16) + gradient;
  int16_t gcolor;
  uint8_t _r,_g,_b;
  if (x1 >= w) x1 = w;
  for (int x = x0 + 1; x <= x1 - 1; x++)
  {
    int16_t _y =  y >> 16;
    if((_y >= 0)&&(_y < h))
    {
      gcolor = (y >> 8)& 255;
      _r = (r * gcolor) >> 8;
      _g = (g * gcolor) >> 8;
      _b = (b * gcolor) >> 8;
      _steepDrawPixelRGB(steep, x, _y, r-_r, g-_g, b-_b);
      _steepDrawPixelRGB(steep, x, _y + 1, _r, _g, _b);
    }
    y += gradient;
  }
}

//Wu's line algorithm, fix point version
void MatrixPanel_DMA::writeLineAA(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) 
{
  int32_t dx = x1 - x0; 
  if (dx < 0) 
  {
    dx = -dx;
  }
  int32_t dy = y1 - y0; 
  if (dy < 0) 
  {
    dy = -dy;
  }
  if (dx == 0) 
  {
    drawFastVLine(x0, min(y0,y1), dy, color);
    return;
  }
  if (dy == 0) 
  {
    drawFastHLine(min(x0,x1), y0, dx, color);
    return;
  }
  if (dy == dx)
  {
    drawLine(x0,y0,x1,y1, color);
    return;
  }

  bool steep = dy > dx;
  if (steep)
  {
    _swap_int16(x0, y0);
    _swap_int16(x1, y1);
  }
  if (x0 > x1)
  {
    _swap_int16(x0, x1);
    _swap_int16(y0, y1);
  }
  uint8_t r,g,b;
  color565to888(color, r, g, b);
  _writeLineAARGB(steep,x0,y0,x1,y1,r,g,b);
}

//Wu's line algorithm, fix point version double wide for all lines
void MatrixPanel_DMA::writeLineAA2(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) 
{
  int32_t dx = x1 - x0; 
  if (dx < 0) 
  {
    dx = -dx;
  }
  int32_t dy = y1 - y0; 
  if (dy < 0) 
  {
    dy = -dy;
  }
  uint8_t r,g,b;
  color565to888(color, r, g, b);
  if (dx == 0) 
  {
    color = color565(r>>1,g>>1,b>>1);
    drawFastVLine(x0, min(y0,y1), dy, color);
    drawFastVLine(x0+1, min(y0,y1), dy, color);
    return;
  }
  if (dy == 0) 
  {
    color = color565(r>>1,g>>1,b>>1);
    drawFastHLine(min(x0,x1), y0, dx, color);
    drawFastHLine(min(x0,x1), y0+1, dx, color);
    return;
  }
  if (dy == dx)
  {
    color = color565(r>>1,g>>1,b>>1);
    drawLine(x0,y0,x1,y1, color);
    drawLine(x0+1,y0,x1+1,y1, color);
    return;
  }

  bool steep = dy > dx;
  if (steep)
  {
    _swap_int16(x0, y0);
    _swap_int16(x1, y1);
  }
  if (x0 > x1)
  {
    _swap_int16(x0, x1);
    _swap_int16(y0, y1);
  }
  _writeLineAARGB(steep,x0,y0,x1,y1,r,g,b);
}