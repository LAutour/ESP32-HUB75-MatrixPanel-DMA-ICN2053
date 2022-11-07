#ifndef ESP32_HUB75_MATRIXPANEL_DMA_ICN2053_H
#define ESP32_HUB75_MATRIXPANEL_DMA_ICN2053_H

#include "ESP32-HUB75-MatrixPanel-DMA-config.h"
#include "ESP32-HUB75-MatrixPanel-DMA-leddrivers.h"
#include "color_convert.h"

//#define NO_FAST_FUNCTIONS

/***************************************************************************************/
/* Library Includes!                                                                   */
//#include <memory>
#include "esp_heap_caps.h"
#include "esp32_i2s_parallel_v2.h"

#if defined(USE_GFX_ROOT)
	#include <FastLED.h>    
	#include "GFX.h" // Adafruit GFX core class -> https://github.com/mrfaptastic/GFX_Root	
#elif !defined(NO_GFX)
  #include "Adafruit_GFX.h" // Adafruit class with all the other stuff
#endif

enum{
  _VIRTUAL_BIT_TOP_DOWN = 1<<0,
  _VIRTUAL_BIT_SERPENTINE = 1<<1,
};

typedef enum{
  VIRTUAL_TOP_DOWN = _VIRTUAL_BIT_TOP_DOWN,
  VIRTUAL_BOTTOM_UP = 0,
  VIRTUAL_S_TOP_DOWN = _VIRTUAL_BIT_SERPENTINE + _VIRTUAL_BIT_TOP_DOWN,
  VIRTUAL_S_BOTTOM_UP = _VIRTUAL_BIT_SERPENTINE,
}virtual_matrix_t;

//определение буфера вывода
typedef struct{
  ESP32_I2S_DMA_STORAGE_TYPE** rowBits; //для ICN2053 - порции данных для вывода (для под 240x120 - приходится экономить память)
  size_t row_data_len; //длина буфера одной DMA строки 
  size_t frame_prefix_len; //длина буфера префикса
  size_t frame_suffix_len; //длина буфера суффикса
  uint8_t all_row_data_cnt; //общее число буферов DMA строк
  uint8_t row_data_cnt; //число DMA строк на кадр
  uint8_t frame_prefix_cnt; //число кадровых префиксов: 0,1
  uint8_t frame_suffix_cnt; //число кадровых суффиксов: 0,1,2
  int8_t color_bits; //глубина цвета на канал
}frameStruct_t;

//константы для первичного фрейм буфера
//для 16-битного буфера битность должна быть 16 или 32
typedef uint32_t vbuffer_t; 
enum{
  VB_SIZE = sizeof(vbuffer_t),
  VB_MBITS = 5,
  VB_MASK = 31,
};

//определение первичного фрейм буфера
typedef struct{
  vbuffer_t* frameBits[2]; //данные буфера
  size_t len; //размер одного буфера
  size_t row_len; //длина строки в vbuffer_t словах
  size_t subframe_len; //размер битового фрейма
  uint8_t subframe_cnt; //число битовых фреймов
}frame_buffer_t; 

//цвет в R8G8B8
typedef struct{
  uint8_t  b;
  uint8_t  g;
  uint8_t  r;
}rgb888_t;

//функция преобразования цвета
//typedef void(*palleteRGB_p)(uint8_t color, rgb888_t& data_rgb);

//пользовательская процедура выдачи цвета пикселя по запросу формирования DMA строки
typedef void(*getUserRGB_p)(int16_t offset_y, int16_t offset_x, rgb888_t& pixel888_high, rgb888_t& pixel888_low);
//пользовательская процедура отрисовки пикселя
typedef void(*drawUserPixel_p)(int16_t x1, int16_t y1, const rgb888_t& color);

/***************************************************************************************/   
#ifdef USE_GFX_ROOT
class MatrixPanel_DMA : public GFX {
#elif !defined NO_GFX
class MatrixPanel_DMA : public Adafruit_GFX {
#else
class MatrixPanel_DMA {
#endif

  // ------- PUBLIC -------
  public:
    // MatrixPanel_DMA
    // default predefined values are used for matrix configuraton
    //параметр для виртуальной панели (число панелей в столбце больше 1)
    //  VIRTUAL_S_TOP_DOWN - начало цепочки панелей вверху, строки панелей соеденены "сепантином"
    //  VIRTUAL_S_BOTTOM_UP - начало цепочки панелей внизу, строки панелей соеденены "сепантином"
    //  VIRTUAL_TOP_DOWN - начало цепочки панелей вверху, строки панелей направлены в одну сторону
    //  VIRTUAL_BOTTOM_UP - начало цепочки панелей внизу, строки панелей направлены в одну сторону 
    MatrixPanel_DMA(virtual_matrix_t _virtual_panel = VIRTUAL_S_TOP_DOWN);
    // MatrixPanel_DMA 
    // @param  hub75_i2s_cfg_t& opts : structure with matrix configuration    
    // 
    MatrixPanel_DMA(const hub75_cfg_t& opts, virtual_matrix_t _virtual_panel = VIRTUAL_S_TOP_DOWN);

    // Propagate the DMA pin configuration, allocate DMA buffs and start data ouput, initialy blank
    bool begin();  
    // overload for compatibility 
    bool begin(hub75_pins_t* hub75_pins_ptr);

    //set rotate write to display
    //rotate: ROTATE_0, ROTATE_90, ROTATE_180, ROTATE_270
    void setRotate(rotate_t _rotate);
    
    // Mirror display
    void setMirrorX(bool _mirror_x);
    void setMirrorY(bool _mirror_y);

    // Adafruit's BASIC DRAW API (565 colour format)
    virtual void drawPixel(int16_t x, int16_t y, uint16_t color);   // overwrite adafruit implementation
    void drawPixelRGB888(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b);
	  virtual void fillScreen(uint16_t color);                        // overwrite adafruit implementation
    void fillScreenRGB888(uint8_t r, uint8_t g, uint8_t b);    
    virtual void clearScreen(); //очистка экрана и буферов
    virtual void setColor(uint16_t color);
    virtual void setColor(uint8_t r, uint8_t g, uint8_t b);
    void setTextColorRGB(uint8_t r, uint8_t g, uint8_t b);
    //рисование сглаженной линии с полутонами
    void writeLineAA(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
    //рисование сглаженной линии с полутонами и двойной толщиной для линий в 45 градусов
    void writeLineAA2(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);

    #ifdef USE_GFX_ROOT
	  // 24bpp FASTLED CRGB colour struct support
	  void fillScreen(CRGB color);
    void drawPixel(int16_t x, int16_t y, CRGB color);
    #endif 

    #ifndef NO_FAST_FUNCTIONS
    virtual void drawFastVLine(int16_t x, int16_t y, int16_t h);
    virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
    virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint8_t r, uint8_t g, uint8_t b);
    virtual void drawFastHLine(int16_t x, int16_t y, int16_t w);
    virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
    virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint8_t r, uint8_t g, uint8_t b);
    virtual void fillRect(int16_t x, int16_t y, int16_t w, int16_t h);    
    virtual void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);    
    virtual void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t r, uint8_t g, uint8_t b);
    virtual void drawRect(int16_t x, int16_t y, int16_t w, int16_t h);
    virtual void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    virtual void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t r, uint8_t g, uint8_t b);
    #endif

    void drawIcon (int* ico, int16_t x, int16_t y, int16_t cols, int16_t rows);
    
    //переключение буфера
    void flipBuffer();    
    //для совместимости
    void flipDMABuffer();

    //this is just a wrapper to control brightness
    //with an 8-bit value (0-255), very popular in FastLED-based sketches :)
    //@param uint8_t b - 8-bit brightness value
    //!!! для ICN2053 потребуется перерисовка кадра по sendFrame(), или flipBuffer()
    void setPanelBrightness(int b);
    void setBrightness8(uint8_t b);

    void invertDisplay(bool negative);

    //Get a class configuration struct
    const hub75_cfg_t& getCfg() const;
        
    //Stop the ESP32 DMA Engine. Screen will forever be black until next ESP reboot.
    void stopDMAoutput();  
    
    //функции для ICN2053

    //отправить кадр из видео буфера (при двойной буферизации будет отправлен выводной буфер)
    //waitSend - ждать окончания вывода кадра по DMA
    //autoVsync - переключить драйвера матрицы на отправленный кадр
    void sendFrame(bool waitSend = false, bool autoVsync = true);
    //переключить драйвера матрицы на отправленный кадр c отправкой одного из регистров конфигурации
    void sendVsync();
    //выключить вывод драйверов матрицы
    void panelShowOn();
    //включить вывод драйверов матрицы
    void panelShowOff();
    //ждать готовность dma для новой отправки данных
    void waitDmaReady();
    //void setpalleteRGB(palleteRGB_p palleteRGB);

  
    //bool autoShowFrame = true; //автовывод отправленного видеобуфера

    //процедура для обрабтчика прерываний DMA
    void sendCallback();
  // ------- PROTECTED -------
  // those might be useful for child classes, like VirtualMatrixPanel
  protected:    
    bool icn2053_clear;//флаг для очистки драйверов экрана без очистки буферов кадров
    hub75_cfg_t m_cfg;// Matrix i2s settings    
    //uint16_t chain_length_x; //количество панелей по X
    //uint16_t chain_length_y; //количество панелей по Y
    uint16_t pixels_per_row;//общая длина строки в пикселях    
    uint16_t CurColor;//текущий цвет rgb565
    rgb888_t CurRGB;    //текущий цвет R8G8B8
    int16_t rows_send_cnt; //счетчик неотправленных строк кадра   
    //uint8_t virtual_panel; //виртуальная панель
    uint8_t virtual_draw;
    bool show_mirror_x = false;
    bool show_mirror_y = false;
    bool change_xy = false;
    bool mirror_x;
    bool mirror_y;
    bool hw_mirror_x = false;
    bool hw_mirror_y = false;
    bool serpentine_chain; // Are we chained? Ain't no party like a... 
    bool top_down_chain;

    //очистка видео буфера (в ключая экран для текущего выводимого буфера)
    //_buff_id - номер буфера:
    //0 - отображаемый, при этом также очищаются драйвера экрана
    //1 - отрисовки (при отключенной двойной буферизации для ICN2053 остается актуальным, так как эти драйвера имеют свой буфер)
    void clearBuffer(uint8_t _buff_id = 1);
    //очистка DMA буфера 
    //_buff_id - номер DMA буфера 
    void clearDmaBuffer(uint8_t _buff_id = 0); 
    //переключение DMA на отправку префикса с сиснхронизацией
    void sendCBVsync();
    //переключение DMA на отправку строки
    void sendCBRow(uint8_t buff_id);
    //отрисовка залитого прямоугольника
    void fillRectBuffer(int16_t x, int16_t y, int16_t w, int16_t h);
    void fillRectBufferVirtual(int16_t x, int16_t y, int16_t w, int16_t h);
    //рисование точки с условным обменом координат
    void _steepDrawPixelRGB(bool steep, int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b);
    void _writeLineAARGB(bool steep, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t r, uint8_t g, uint8_t b);

   // ------- PRIVATE -------
  private:    
    //омновные буфера
    uint8_t  rows_per_frame;//число строк по адресным линиям модуля матрицы
    frameStruct_t dma_buff; //видеобуфер\буфер вывода для ICN2053
    frame_buffer_t frame_buffer; //первичный видеобуфер для ICN2053
    lldesc_t* dmadesc_data[2]; //дескрипторы видео буферов\буфера вывода для ICN2053
    lldesc_t* dmadesc_prefix; //дескриптор префикса для ICN2053 для управления регистрами драйвера и вертикальной синхронизации (сброс счетчика строк)
    lldesc_t* dmadesc_suffix[2]; //дескрипторы регенерация экрана для ICN2053\регистров драйвера ICN2038
    lldesc_t* dmadesc_ext; //дескрипторы переходов в суффиксы для ICN2053 - обеспечивают непрерывную регенерацию экрана
    uint8_t  cur_suffix_id;
    uint16_t prefix_to_suffix; //индекс суффикса после префикса
    uint16_t data_to_suffix; //индекс суффикса после данных строки
    uint16_t desc_data_cnt;   //число дескрипторов для данных строк на каждый каждый видеобуфер
    uint16_t desc_prefix_cnt; //число дескрипторов для кадрового префика 
    uint16_t desc_suffix_cnt; //число дескрипторов для кадрового суффикса на каждый суффикс

    //для инициализации драйверов
    uint16_t driver_cnt; //число драйверов-регистров (оптимизация вычислений);
    driver_rgb_t* driver_reg; //копия регистров конфигурации для драйверов (инициализация числа строк, яркости и т.д.)
    uint8_t driver_cur_reg; //индекс текущего регистра конфигурации

    int dma_int_cnt;//счетчик прерываний DMA - нужен как костыль для счета прерываний DMA пересылок через ОС
    bool bufferReady; //готовность видеобуфера (не зянят и было прерыывание окончания отсылки DMA буфера)
    bool bufferBusy; //занятость видеобуфера (запрет на установку флага bufferReady и смены видеобуфера)



    // Other private variables
    bool initialized; //готовность дисплея
    uint8_t back_buffer_id; //текущий вторичный буфер при двойной буферизации\текущий суффикс для ICN2038
    uint8_t brightness; //яркость экрана от 0 до 255 (0..100%) (в принципе переменная не нужная, если не выставлять яркость до инициализации буферов)
    uint16_t* brightness_table; //таблица гамммы\яркости для преобразования яркости каналов RGB

    //вывод изображения в негативе
    bool negative_panel;
    //palleteRGB_p userPalleteRGB = NULL;

    //быстрый рулонный сдвиг по Y для ICN2053 - до конца не доработан ()
    int32_t scroll_y;
    //быстрый рулонный сдвиг по X для ICN2053 - только для прямой цепочки панелей
    int32_t scroll_x;

    //пользовательская процедура выдачи цвета пикселя по запросу формирования DMA строки
    getUserRGB_p getUserRGB;  
    //пользовательская процедура отрисовки пикселя
    drawUserPixel_p drawUserPixel;
    //автоматическое переключение изображения после отправки видеострок в драйвера icn2053
    //иначе нужно вручную вызывать sendVsync(), при этом можно вывести заранее отправленное изображение с минимальной задержкой
    bool icn2053_auto_vsync;

    //----- общие функции  -----
    //очистка выделенной памяти  
    void buffersFree(); 
    //выделение памяти
    bool allocateDMAmemory();
    //инициализация буферов и настройка DMA
    void configureDMA();

    //----- функции icn2053 -----
    //инициализация буферов вывода
    void icn2053initBuffers();
    //первичная инициализация драйверов экрана (отправка всех конфигурационных регистров)
    void icn2053init();
    //заполнение префикса данными конфигурационного регистра
    void icn2053setReg(uint8_t reg_idx, driver_rgb_t* regs_data);
    //вкл\выкл экрана и смкены кадра в префиксе
    void icn2053setVSync(bool leds_enable, bool vsync);

    //установка пользовательской процедуры выдачи цвета пикселя по запросу формирования DMA строки
    void setDMAGetUserRGB(getUserRGB_p _getUserRGB);  
    //пользовательская процедура отрисовки пикселя
    void setDrawUserPixel(drawUserPixel_p _drawUserPixel);  

    //отрисовка прямоугольника в видеобуфере
    void fillRectFrameBuffer(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
    
    //----- функции shift и icn2038 -----
    #ifdef DIRECT_DRIVER_INIT
    //инициализация буферов с непосредстенной запись с порт - оставлено для отладкиы    
    void shiftDriverInit();
    #endif

    #ifdef USE_COLORx16
    //получить цвет пикселя из видеобуфера
    void getRGBColor16(int offset_y, int offset_x, rgb888_t& rgb888);
    #endif
    //заполнение DMA буфера строки
    void prepareDmaRows(uint8_t row_offset, uint8_t dma_buff_id);        
}; // end Class header

#endif
