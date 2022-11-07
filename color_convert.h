#ifndef _COLOR_CONVERT
#define _COLOR_CONVERT

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
  COLORx1 = 1,      //2 цвета через палитру (только для ICN2053)
  COLORx2 = 2,      //4 цвета через палитру (только для ICN2053)
  COLOR111 = 3,     //8 цветов 3x1 битпланов
  COLORx3 = 3,      //8 цветов 3x1 битпланов
  COLORx4 = 4,      //16 цветов через палитру (только для ICN2053)
  COLOR222 = 6,     //64 цвета
  COLORx8 = 8,      //256 цветов через палитру (только для ICN2053)
  COLOR333 = 9,     //512 цветов
  COLOR444 = 12,    //4096 цветов
  COLOR555 = 15,    //32768 цветов
  COLORx16 = 16,    //HICOLOR (только для ICN2053)
  COLOR565 = 16,    //HICOLOR (только для ICN2053)
  COLOR666 = 18,    
  COLOR777 = 21,
  COLORx24 = 24,    //TRUECOLOR 3x8 битпланов
  COLOR888 = 24,    //TRUECOLOR 3x8 битпланов
  //COLORx48 = 48,    //в теории DEEPCOLOR 3x16 битпланов (только для ICN2053)
}color_depth_t;

#define color888to565(r,g,b) color565(r,g,b)

//extern uint16_t Translate8To16Bit[256];
extern const uint8_t lumConvTab[256];
enum{BRIGHT_TABLE_SIZE = sizeof(lumConvTab)};

// Converts RGB111 to RGB565
uint16_t color111to565(uint8_t r, uint8_t g, uint8_t b);
// Converts RGB222 to RGB565
uint16_t color222to565(uint8_t r, uint8_t g, uint8_t b);
// Converts RGB333 to RGB565
uint16_t color333to565(uint8_t r, uint8_t g, uint8_t b);
// Converts RGB444 to RGB565
uint16_t color444to565(uint8_t r, uint8_t g, uint8_t b);
// Converts RGB555 to RGB565
uint16_t color555to565(uint8_t r, uint8_t g, uint8_t b);
// Converts R6G6B6 to RGB565
uint16_t color666to565(uint8_t r, uint8_t g, uint8_t b);
// Converts R7G7B7 to RGB565
uint16_t color777to565(uint8_t r, uint8_t g, uint8_t b);
// Converts R8G8B8 to RGB565
uint16_t color565(uint8_t r, uint8_t g, uint8_t b);
// Converts RXGXBX to RGB565
uint16_t colorXXXto565(uint8_t r, uint8_t g, uint8_t b, uint8_t X); 
// Converts RGB565 to RGB888
void color565to888(const uint16_t color, uint8_t &r, uint8_t &g, uint8_t &b);

// Converts packet ColorXXX to RXGXBX
void colorXXXtoRXGXBX(uint16_t colorXXX, uint8_t &r, uint8_t &g, uint8_t &b, uint8_t X);
// Converts RXGXBX to packet ColorXXX
uint16_t colorRXGXBXtoXXX(uint8_t r, uint8_t g, uint8_t b, uint8_t X);



#endif

