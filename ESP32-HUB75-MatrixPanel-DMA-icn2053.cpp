#include <Arduino.h>
#include "ESP32-HUB75-MatrixPanel-DMA-icn2053.h"
#include "ESP32-HUB75-MatrixPanel-DMA-leddrivers.h"

#ifndef DISABLE_SERIAL_DEBUG
  #define SERIAL_DEBUG
#endif 

#define offset_prefix  dma_buff.all_row_data_cnt
#define offset_suffix  (dma_buff.all_row_data_cnt + dma_buff.frame_prefix_cnt)
/***************************************************************************************/   

#ifndef NO_CIE1931
#endif

typedef struct {
  int16_t x;
  int16_t y;
}coords_t;

// Max clock cycles to blank OE before/after LAT signal change
//enum{CLKS_DURING_LATCH = 0};

//для callback обработчика
MatrixPanel_DMA* MatrixPanel = NULL;

//очистка буферов при неудачной инициализации 
void MatrixPanel_DMA::buffersFree()
{
  int i;
  if (dma_buff.rowBits != NULL)
  {
    i = dma_buff.all_row_data_cnt + dma_buff.frame_prefix_cnt + dma_buff.frame_suffix_cnt;
    while (i > 0)
    {
      i--;
      if (dma_buff.rowBits[i] != NULL)
      {
        heap_caps_free(dma_buff.rowBits[i]);
      }
    }
    heap_caps_free(dma_buff.rowBits);
  }
  for(i = m_cfg.double_buff-1; i >= 0; i--)
  {
    if(frame_buffer.frameBits[i] != NULL)
    {
      heap_caps_free(frame_buffer.frameBits[i]);
    }
  }
  if (dmadesc_data[0] != NULL)
  {
    heap_caps_free(dmadesc_data[0]);
  }
  if (brightness_table != NULL)
  {
    heap_caps_free(brightness_table);
  }

}

//---------------------------//
//  Функции вывода в панели  //
//---------------------------//

//получение цвета пиксела в фрейм-буфере
#ifdef USE_COLORx16
inline void MatrixPanel_DMA::getRGBColor16(int offset_y, int offset_x, rgb888_t& rgb888)
{
  uint16_t* addr = (uint16_t*)&frame_buffer.frameBits[back_buffer_id^m_cfg.double_buff][offset_y];
  color565to888(addr[offset_x], rgb888.r,rgb888.g, rgb888.b);
}
#endif

//подготовка DMA-буфера
IRAM_ATTR void MatrixPanel_DMA::prepareDmaRows(uint8_t row_offset, uint8_t dma_buff_id)
{
  rgb888_t pixel888_1;
  rgb888_t pixel888_2;
  driver_rgb_t pixel1;
  driver_rgb_t pixel2;

  if (icn2053_clear) return;

  row_offset = (rows_per_frame - row_offset) + scroll_y;// - dma_buff.row_data_cnt;// + scroll_y;
  //row_offset
  for(int row_buf_id = 0; row_buf_id < dma_buff.row_data_cnt; row_buf_id++)
  {
    //рулонный сдвиг по y
    if (row_offset >= rows_per_frame) row_offset -= rows_per_frame;
    ///смещение в строку видеобуфера
    int offset_y = row_offset*frame_buffer.row_len;

    //смещения для буфера вывода
    int dma_buff_offset = 0;//DRIVER_BITS;
    int  dma_buff_offset_next = 0;//DRIVER_BITS;
    int driver_idx = DRIVER_BITS;
    int offset_x;
    if (hw_mirror_x)
    {
      //рулонный сдвиг по x
      offset_x = pixels_per_row + scroll_x - 1;
      if (offset_x >= pixels_per_row) offset_x -= pixels_per_row;
    }else
    {
      //рулонный сдвиг по x
      offset_x = 0 - scroll_x;
      if (offset_x < 0 ) offset_x += pixels_per_row;    
    }
    //индекс строки в DMA буфере
    //дл одиночного буфера dma_buff_id = 0 
    int row_dma_offset = (dma_buff_id)?(row_buf_id + dma_buff.row_data_cnt):row_buf_id;

    int8_t color_bits = dma_buff.color_bits;
    for (int i = pixels_per_row; i > 0; i--)
    { 
      switch (color_bits)
      {
        #ifdef USE_COLORx16
        case -16:
          getRGBColor16(offset_y, offset_x, pixel888_1);
          getRGBColor16(offset_y + (frame_buffer.subframe_len >> 1), offset_x, pixel888_2);
        break;
        #endif
        default:
          if (getUserRGB != NULL) getUserRGB(row_offset, offset_x, pixel888_1, pixel888_2);
        break;
      }

      if (negative_panel) 
      {
        pixel888_1.r = ~pixel888_1.r;
        pixel888_1.g = ~pixel888_1.g;
        pixel888_1.b = ~pixel888_1.b;
        pixel888_2.r = ~pixel888_2.r;
        pixel888_2.g = ~pixel888_2.g;
        pixel888_2.b = ~pixel888_2.b;
      }

      pixel1.r = brightness_table[pixel888_1.r];
      pixel1.g = brightness_table[pixel888_1.g];  
      pixel1.b = brightness_table[pixel888_1.b];  
      pixel2.r = brightness_table[pixel888_2.r];
      pixel2.g = brightness_table[pixel888_2.g];  
      pixel2.b = brightness_table[pixel888_2.b];  

      //вывод пиксела в видеобуфер
      ESP32_I2S_DMA_STORAGE_TYPE mask_rgb12;
      for (int j = DRIVER_BITS-1; j >= 0; j--)
      {
        mask_rgb12 = 0;
        if (pixel1.r < 0) mask_rgb12 |= BIT_R1; pixel1.r <<= 1;
        if (pixel1.g < 0) mask_rgb12 |= BIT_G1; pixel1.g <<= 1;
        if (pixel1.b < 0) mask_rgb12 |= BIT_B1; pixel1.b <<= 1;
        if (pixel2.r < 0) mask_rgb12 |= BIT_R2; pixel2.r <<= 1;
        if (pixel2.g < 0) mask_rgb12 |= BIT_G2; pixel2.g <<= 1;
        if (pixel2.b < 0) mask_rgb12 |= BIT_B2; pixel2.b <<= 1;

        int offset_dma = dma_buff_offset ^ 1;        
        dma_buff.rowBits[row_dma_offset][offset_dma] = (dma_buff.rowBits[row_dma_offset][offset_dma] & (~BITMASK_RGB12))|mask_rgb12;
        dma_buff_offset++;
      }
      driver_idx--;
      if (driver_idx == 0)
      {
        driver_idx = DRIVER_BITS;
        dma_buff_offset_next += DRIVER_BITS; 
        dma_buff_offset = dma_buff_offset_next;
      }else
      {
        dma_buff_offset += pixels_per_row + SUBROW_ADD_LEN - DRIVER_BITS;  
      }
      if (hw_mirror_x)
      {
        if (offset_x == 0) offset_x = pixels_per_row - 1;
        else offset_x--;
      }else
      {
        offset_x++;
        if (pixels_per_row == 0) offset_x = 0;
      }
    }
    row_offset++;
  }
}

//выключить вывод драйверов панели
void MatrixPanel_DMA::panelShowOff()
{
  if (m_cfg.driver == ICN2053)
  {
    //можно проще, но тогда нужно городить еще DMA буфер и переключение на него только для одной команды
    waitDmaReady();
    icn2053setVSync(false, false);
    sendVsync();
    waitDmaReady();
    icn2053setVSync(false, true);
  }else
  {
    //shift
  }
}

//включить вывод драйеров панели
void MatrixPanel_DMA::panelShowOn()
{
  if (m_cfg.driver == ICN2053)
  {
    //можно проще, но тогда нужно городить еще DMA буфер и переключение на него только для одной команды
    waitDmaReady();
    icn2053setVSync(true, false);
    sendVsync();
    waitDmaReady();
    icn2053setVSync(true, true);    
  }else
  {
    //shift
  }
}

//подготовка префикса и переключение DMA вывода
IRAM_ATTR void MatrixPanel_DMA::sendCBVsync()
{
  icn2053setReg(driver_cur_reg, (&driver_reg[driver_cur_reg]));
  driver_cur_reg++;
  if (driver_cur_reg >= ICN2053_REG_CNT) driver_cur_reg = 0;

  //префикс на бак-суффикс
  dmadesc_ext[ICN2053_EXT_PREFIX].qe.stqe_next = &dmadesc_suffix[cur_suffix_id][prefix_to_suffix];
  //закольцовываем бак-суффикс
  dmadesc_suffix[cur_suffix_id][desc_suffix_cnt-1].qe.stqe_next = dmadesc_suffix[cur_suffix_id];
  dmadesc_suffix[cur_suffix_id][desc_suffix_cnt-1].eof = true;
  //меняем cur_suffix_id
  cur_suffix_id ^= 1;
  //фронт-суффикс на префикс
  dmadesc_suffix[cur_suffix_id][desc_suffix_cnt-1].qe.stqe_next = dmadesc_prefix;  
  //ждем прерывания окончания передачи фронт-суффикса и префикса
  dma_int_cnt = -1;
  //dmadesc_suffix[cur_suffix_id][desc_suffix_cnt-1].eof = true;
}
  
//переключение DMA вывода на строку
IRAM_ATTR void MatrixPanel_DMA::sendCBRow(uint8_t buff_id)
{
  //данные строки на бак-суффикс
  dmadesc_ext[ICN2053_EXT_DATA].qe.stqe_next = &dmadesc_suffix[cur_suffix_id][data_to_suffix];
  //закольцовываем бак-суффикс
  dmadesc_suffix[cur_suffix_id][desc_suffix_cnt-1].qe.stqe_next = dmadesc_suffix[cur_suffix_id];
  dmadesc_suffix[cur_suffix_id][desc_suffix_cnt-1].eof = true;
  //меняем cur_suffix_id
  cur_suffix_id ^= 1;
  //фронт-суффикс на данные строки;
  dmadesc_suffix[cur_suffix_id][desc_suffix_cnt-1].qe.stqe_next = dmadesc_data[buff_id];
  //ждем прерывания окончания передачи фронт-суффикса и данных строки
  dma_int_cnt = -1;
  //dmadesc_suffix[cur_suffix_id][desc_suffix_cnt-1].eof = true;
}

//обработчик вывода строк
IRAM_ATTR void MatrixPanel_DMA::sendCallback()
{
  static uint8_t dma_buffer_id = 0;
  if (bufferBusy || bufferReady || dma_int_cnt > 0) return;
  bufferBusy = true;
  dma_int_cnt++;

  if (dma_int_cnt > 0) 
  {
    dmadesc_suffix[cur_suffix_id^1][desc_suffix_cnt-1].eof = false;
    //dma_int_cnt = 0;  

    //Serial.print(rows_send_cnt);
    if (rows_send_cnt > 0)
    {      
      //int row_send_idx = rows_send_cnt-1;
      //отправка строк DMA буфера   
      //m_cfg.double_dma_buff = false;
      if (m_cfg.double_dma_buff)
      {         
        //двойной буфер строк
        if (rows_send_cnt == rows_per_frame) 
        {
          //готовим первый DMA буфер
          prepareDmaRows(rows_send_cnt, dma_buffer_id);
          //выводим первый DMA буфер
          sendCBRow(dma_buffer_id);
          dma_buffer_id ^= 1;
        }else 
        {
          //выводим другой DMA буфер
          sendCBRow(dma_buffer_id^1);
        }
        if (rows_send_cnt > dma_buff.row_data_cnt) 
        {
          //готовим очередной DMA буфер
          prepareDmaRows(rows_send_cnt - dma_buff.row_data_cnt, dma_buffer_id);
          dma_buffer_id ^= 1;
        }        
      }else
      {
        //одиночный буфер строк
        //готовим строку
        prepareDmaRows(rows_send_cnt, 0);
        //выводим строку
        sendCBRow(0);
      }

      rows_send_cnt -= dma_buff.row_data_cnt; 
      if (rows_send_cnt <= 0) 
      {
        icn2053_clear = false;
        //автовывод изобржения при icn2053_auto_vsync = 1
        if (icn2053_auto_vsync) rows_send_cnt = 0;
        else rows_send_cnt = -1;
        dma_int_cnt = -4; //костыль при DMA_CLOCK > 5Mhz- надо разбираться
      }
    }else if (rows_send_cnt == 0)
    {
      //отправка синхры после отправки строк DMA буфера
      sendCBVsync();
      //Serial.print("*");
      rows_send_cnt = -1;
    }else
    {
      //DMA свободен
      bufferReady = true; 
    }
  }
  bufferBusy= false;
}

//обработчик прерывания от DMA
//разрешается в начале DMA посылки (EOF)
//срабатывает в конце посылки
IRAM_ATTR void icn2053i2sCallback()
{ 
  MatrixPanel->sendCallback();
}


//ожидание готовности DMA
void MatrixPanel_DMA::waitDmaReady()
{
  if (frame_buffer.len == 0) return;
  while(!bufferReady)
  {
    delay(1);
  }  
}

//вывод VSYNC (комнад и регистров управления драйверами)
void MatrixPanel_DMA::sendVsync()
{
  waitDmaReady();
  bufferReady = false; //резервируем DMA
  bufferBusy = true; //маскируем обработчик вывода строк  
  sendCBVsync();  
  bufferBusy = false;

}

//запуск отправки фрейма в back-буфер драйверов
void MatrixPanel_DMA::sendFrame(bool waitSend, bool autoVsync)
{
  waitDmaReady();
  //Serial.print(">");
  //Serial.print(m2-m1);Serial.print(" ");
  bufferReady = false; //резервируем DMA
  dma_int_cnt = 0;
  icn2053_auto_vsync = autoVsync;
  dmadesc_suffix[cur_suffix_id^1][desc_suffix_cnt-1].eof = true; //разрешаем прерывания
  rows_send_cnt = rows_per_frame; //инициализируем счетчик вывода строк
  //Serial.print(">");Serial.println(back_buffer_id^1);
  if (waitSend) waitDmaReady();
}

//инициализация регистров управления драйверами с очисткой экрана
void MatrixPanel_DMA::icn2053init()
{
  icn2053setVSync(false, true); //запретить вывод изображения
  for (int i = 0; i < ICN2053_REG_CNT; i++)
  {
    sendVsync();
    //Serial.println(F("4"));
  }
  //один буфер драйвера
  icn2053_clear= true;
  sendFrame();
  //второй буфер драйвера
  icn2053_clear= true;
  sendFrame();  
  icn2053setVSync(true, true); //разрешить вывод изображения
  sendVsync();
}

//---------------------------------//
//  Функции инициализации буферов  //
//---------------------------------//

//выделеение памяти
bool MatrixPanel_DMA::allocateDMAmemory()
{
  /***
  * Step 1: Look at the overall DMA capable memory for the DMA FRAMEBUFFER data only (not the DMA linked list descriptors yet) 
  *         and do some pre-checks.
  */
  //size_t row_data_buffer_size;
  size_t matrix_buffer_size = 0;
  size_t subrow_data_len = pixels_per_row + SUBROW_ADD_LEN; //длина подстроки
  size_t gamma_table_len = 0;
  uint8_t dma_buff_cnt; //общее количество DMA буферов

  m_cfg.double_buff &= 1;
  m_cfg.double_dma_buff &= 1;
  if (m_cfg.driver == ICN2053)
  { 
    //двойной видеобуфер встроен в драйвер матрицы, переключение по V_SYNC c инициализацией одного из регистров    
    //переключение буфера драйвера матрицы совмещеено с передайче регистров
    back_buffer_id = m_cfg.double_buff;
    gamma_table_len = BRIGHT_TABLE_SIZE;

    //буфер префикса   
    dma_buff.frame_prefix_len = FRAME_ADD_LEN + ICN2053_PREFIX_START_LEN + pixels_per_row;
    dma_buff.frame_prefix_cnt = ICN2053_PREFIX_CNT; //V_SYNC + уствновка регистров
    //буфера суффикса
    dma_buff.frame_suffix_len = ICN2053_ROW_OE_LEN*rows_per_frame; //*2 - фронт+спад    
    dma_buff.frame_suffix_cnt = ICN2053_SUFFIX_CNT; //перебор строк
    //буфера строк   
    dma_buff.row_data_len = subrow_data_len*DRIVER_BITS + ROW_ADD_LEN;
    //размер первичного видеобуфера и числа строк данных
    dma_buff.row_data_cnt = ICN2053_DMA_ROW_BUFF_CNT;
    if (dma_buff.row_data_cnt > rows_per_frame) dma_buff.row_data_cnt = rows_per_frame;
 
    dma_buff.color_bits = -m_cfg.color_depth;
    //требуется промежуточный видеобуфер без требования DMA
    #ifdef SERIAL_DEBUG
		Serial.printf_P(PSTR("Color depth id = %d\r\n"), dma_buff.color_bits);
    #endif
      
    switch (dma_buff.color_bits)
    {
      #ifdef USE_COLORx16
      case -16:
        //двухбайтовое поле
        frame_buffer.row_len = (pixels_per_row*2 + VB_SIZE - 1)/VB_SIZE;
        frame_buffer.subframe_len = frame_buffer.row_len*m_cfg.mx_height;
        frame_buffer.len = frame_buffer.subframe_len;
      break;
      #endif
      default:
        //обработка буфера на стороне пользователя
        frame_buffer.row_len = 0;
      break; 
    }
    //начало вычисления размера общего буфера
    dma_buff_cnt = ICN2053_PREFIX_CNT + ICN2053_SUFFIX_CNT;
    matrix_buffer_size = (dma_buff.frame_prefix_len*ICN2053_PREFIX_CNT + dma_buff.frame_suffix_len*ICN2053_SUFFIX_CNT)*SIZE_DMA_TYPE;
    matrix_buffer_size += (frame_buffer.len*VB_SIZE) << m_cfg.double_buff;
  }else //if (m_cfg.driver == ICN2038S)
  {
    Serial.println("Support SHIFT drivers - disabled!"); 
    buffersFree();
    return false;
  }
  dma_buff.all_row_data_cnt = dma_buff.row_data_cnt << m_cfg.double_dma_buff;
  dma_buff_cnt += dma_buff.all_row_data_cnt;
  matrix_buffer_size += dma_buff.row_data_len*dma_buff.all_row_data_cnt*SIZE_DMA_TYPE + gamma_table_len*sizeof(uint16_t);
  
  size_t heap_free_size = heap_caps_get_free_size(MALLOC_CAP_DMA);
	// 1. Calculate the amount of DMA capable memory that's actually available
  #ifdef SERIAL_DEBUG    
  if (m_cfg.double_dma_buff) 
  {
    Serial.println(F("DOUBLE FRAME BUFFERS / DOUBLE BUFFERING IS ENABLED. DOUBLE THE RAM REQUIRED!"));
  }        
  Serial.println(F("DMA memory blocks available before any malloc's: "));
  heap_caps_print_heap_info(MALLOC_CAP_DMA);
	Serial.println(F("******************************************************************"));
  Serial.printf_P(PSTR("We're going to need %d bytes of SRAM just for the frame buffer(s).\r\n"), matrix_buffer_size);    
  Serial.printf_P(PSTR("The total amount of DMA capable SRAM memory is %d bytes.\r\n"), heap_free_size);
  Serial.printf_P(PSTR("Largest DMA capable SRAM memory block is %d bytes.\r\n"), heap_caps_get_largest_free_block(MALLOC_CAP_DMA)); 
  if (m_cfg.driver == ICN2053)          
	Serial.println(F("******************************************************************"));	    		
  #endif

  // Can we potentially fit the framebuffer into the DMA capable memory that's available?
  if ( heap_free_size < matrix_buffer_size  ) 
  {      
    #ifdef SERIAL_DEBUG      
    Serial.printf_P(PSTR
      ("######### Insufficient memory for requested resolution. Reduce MATRIX_COLOR_DEPTH and try again.\r\n\tAdditional %d bytes of memory required.\r\n\r\n"), 
      (matrix_buffer_size-heap_caps_get_free_size(MALLOC_CAP_DMA)) );
    #endif
    return false;
  }
  dma_buff.rowBits = (ESP32_I2S_DMA_STORAGE_TYPE**)heap_caps_calloc(dma_buff_cnt,sizeof(void*), MALLOC_CAP_32BIT | MALLOC_CAP_DMA);
  if (dma_buff.rowBits == NULL)
  {
    #ifdef SERIAL_DEBUG
		Serial.printf_P(PSTR("ERROR: Couldn't malloc frameBits ptrs %d! Critical fail.\r\n"), (uint32_t)dma_buff.rowBits);
    #endif
    return false;
  }

  size_t data_len = dma_buff.row_data_len;
  //выделяем DMA буфер на строки
	for (int malloc_num = 0; malloc_num < dma_buff_cnt; malloc_num++)
	{
    #ifdef SERIAL_DEBUG
		Serial.print("Malloc memory for ");
    #endif
    //тут оптимизация под то, что служебных буферов не более двух
    if (malloc_num >= dma_buff.all_row_data_cnt)
    {
      if (malloc_num < (dma_buff.all_row_data_cnt + dma_buff.frame_prefix_cnt))
      {
        data_len = dma_buff.frame_prefix_len;
        #ifdef SERIAL_DEBUG
		    Serial.print("Prefix");
        #endif
      }
      else 
      {
        data_len = dma_buff.frame_suffix_len;
        #ifdef SERIAL_DEBUG
		    Serial.print("Suffix");
        #endif
      }
    }else
    {
      #ifdef SERIAL_DEBUG
		  Serial.print("Row");
      #endif
    }
    dma_buff.rowBits[malloc_num] = (ESP32_I2S_DMA_STORAGE_TYPE*)heap_caps_calloc(data_len, SIZE_DMA_TYPE,  MALLOC_CAP_DMA);

    if (dma_buff.rowBits[malloc_num] == NULL)
    {
      #ifdef SERIAL_DEBUG
		  Serial.printf_P(PSTR(" - ERROR: Couldn't malloc DMA buffer %d! Critical fail.\r\n"), malloc_num);
      #endif
      buffersFree();
    	return false;
    }
    #ifdef SERIAL_DEBUG
    Serial.printf_P(PSTR(": %d bytes @ address %u(d) (DMA buffer %d).\r\n"), data_len*SIZE_DMA_TYPE, (uint32_t)dma_buff.rowBits[malloc_num], malloc_num);
    #endif
	}
  //выделяем не DMA первичный буфер для ICN2053
  if (frame_buffer.len > 0)
  {
    for (int frame_id = m_cfg.double_buff; frame_id >= 0; frame_id--)
    {
      #ifdef SERIAL_DEBUG
		  Serial.printf_P(PSTR("Malloc memory for primary framebuffer %d"), frame_id);
      #endif

      frame_buffer.frameBits[frame_id] = (vbuffer_t*)heap_caps_calloc(1, frame_buffer.len*VB_SIZE, MALLOC_CAP_8BIT);
      if (frame_buffer.frameBits[frame_id] == NULL)
      {
      #ifdef SERIAL_DEBUG
		    Serial.printf_P(PSTR(" - ERROR: Couldn't malloc frame_buffer! Critical fail.\r\n"));
        #endif
        buffersFree();
    	  return false;
      }
      #ifdef SERIAL_DEBUG
      Serial.printf_P(PSTR(": %d bytes @ address %u(d) (no DMA buffer).\r\n"), frame_buffer.len*VB_SIZE, (uint32_t)frame_buffer.frameBits[frame_id]);
      #endif
    }
  }
  if (gamma_table_len > 0)
  {
    #ifdef SERIAL_DEBUG
		Serial.println(PSTR("Malloc memory for gamma-color - 512b"));
    #endif
    brightness_table = (uint16_t*)heap_caps_calloc(1, gamma_table_len*sizeof(uint16_t), MALLOC_CAP_8BIT);    
    if (brightness_table == NULL)
    {
      #ifdef SERIAL_DEBUG
		  Serial.printf_P(PSTR(" - ERROR: Couldn't malloc gamma-color buffer! Critical fail.\r\n"));
      #endif
      buffersFree();
    	return false;
    }    
  }

  //количество DMA дескрипторов
  desc_data_cnt = (dma_buff.row_data_len*SIZE_DMA_TYPE + DMA_MAX-1)/DMA_MAX; //дескрипторов на строку
  desc_data_cnt *= dma_buff.row_data_cnt; //дескрипторов на выводимые строки (один буфер)
  desc_prefix_cnt = (dma_buff.frame_prefix_len*SIZE_DMA_TYPE + DMA_MAX-1)/DMA_MAX;  
  desc_prefix_cnt *= dma_buff.frame_prefix_cnt;
  desc_suffix_cnt = (dma_buff.frame_suffix_len*SIZE_DMA_TYPE + DMA_MAX-1)/DMA_MAX;  
  desc_suffix_cnt *= dma_buff.frame_suffix_cnt;

  //резервирование DMA дескрипторов
  int desc_cnt = desc_suffix_cnt;
  int desc_ext_cnt = 0;
  
  if (m_cfg.driver == ICN2053)
  { 
    desc_ext_cnt = ICN2053_DESCEXT_CNT; //точки входа с продолжением адресов строк     
    desc_cnt *= ICN2053_DSUFFIX_CNT;
  }else if (m_cfg.driver == ICN2038)
  {
    desc_suffix_cnt /= ICN2038_DSUFFIX_CNT; //число дескрипторов на 1 суффикс
  }
  #ifdef SERIAL_DEBUG
  Serial.printf_P(PSTR("Descriptors rows count: %d\r\n"), desc_data_cnt << m_cfg.double_dma_buff);
  Serial.printf_P(PSTR("Descriptors prefix count: %d\r\n"), desc_prefix_cnt);
  Serial.printf_P(PSTR("Descriptors suffix count: %d\r\n"), desc_cnt);
  Serial.printf_P(PSTR("Descriptors ext count: %d\r\n"), desc_ext_cnt);
  #endif
  desc_cnt += (desc_data_cnt<< m_cfg.double_dma_buff) + desc_prefix_cnt + desc_ext_cnt; //всего дескрипторов

  #ifdef SERIAL_DEBUG 
  //Serial.printf_P(PSTR("All DMA desk count = %d\r\n"),desc_cnt);
  #endif

  //выделяем общий буфер на дескрипторы
  dmadesc_data[0] = (lldesc_t *)heap_caps_calloc(desc_cnt, sizeof(lldesc_t), MALLOC_CAP_DMA);
  if(dmadesc_data[0] == NULL) 
  {
    #ifdef SERIAL_DEBUG 
    Serial.println(("ERROR: Could not malloc descriptors"));
    #endif
    buffersFree();
    return false;
  }
  size_t _dma_capable_memory_reserved = desc_cnt*sizeof(lldesc_t);
  #ifdef SERIAL_DEBUG
  Serial.printf_P(PSTR("Descriptors memory reserved: %d\r\n"), _dma_capable_memory_reserved);
  #endif

  //распределяем дескрипторы
  //строк
  if (m_cfg.double_dma_buff) dmadesc_data[1] = &dmadesc_data[0][desc_data_cnt]; else dmadesc_data[1] = dmadesc_data[0];
  //префикса
	if (dma_buff.frame_prefix_cnt > 0)dmadesc_prefix = &dmadesc_data[1][desc_data_cnt]; else dmadesc_prefix = &dmadesc_data[0][0];
  //суффиксов
  dmadesc_suffix[0] = &dmadesc_prefix[desc_prefix_cnt];    
  if ((m_cfg.driver == ICN2038)||(m_cfg.driver == ICN2053))
  {
    dmadesc_suffix[1] = &dmadesc_suffix[0][desc_suffix_cnt];
    if (m_cfg.driver == ICN2053)
    {
      dmadesc_ext = &dmadesc_suffix[1][desc_suffix_cnt];    
    }
  }

  #ifdef SERIAL_DEBUG
  Serial.printf_P(PSTR("Descriptors data[0] address: %u(d), count: %d\r\n"), (uint32_t)dmadesc_data[0], desc_data_cnt);
  Serial.printf_P(PSTR("Descriptors data[1] address: %u(d), count: %d\r\n"), (uint32_t)dmadesc_data[1], desc_data_cnt);
  Serial.printf_P(PSTR("Descriptors prefix address: %u(d), count: %d\r\n"), (uint32_t)dmadesc_prefix, desc_prefix_cnt);
  Serial.printf_P(PSTR("Descriptors suffix[0] address: %u(d), count: %d\r\n"), (uint32_t)dmadesc_suffix[0], desc_suffix_cnt); 
  Serial.printf_P(PSTR("Descriptors suffix[1] address: %u(d), count: %d\r\n"), (uint32_t)dmadesc_suffix[1], desc_suffix_cnt);
  Serial.printf_P(PSTR("Descriptors ext address: %u(d), count: %d\r\n"), (uint32_t)dmadesc_ext, desc_ext_cnt);
  #endif


  #ifdef SERIAL_DEBUG
  _dma_capable_memory_reserved += matrix_buffer_size;
  Serial.println(F("*** ESP32-HUB75-MatrixPanel-I2S-DMA: Memory Allocations Complete ***"));
  Serial.printf_P(PSTR("Total DMA memory that was reserved: %dkB,%dB .\r\n"), _dma_capable_memory_reserved/1024, _dma_capable_memory_reserved%1024);
  Serial.printf_P(PSTR("Total NODMA memory that was reserved: %dkB,%dB.\r\n"), ((frame_buffer.len+gamma_table_len)*VB_SIZE)/1024, ((frame_buffer.len+gamma_table_len)*VB_SIZE)%1024);
  //Serial.printf_P(PSTR("... of which was used for the DMA Linked List(s): %d kB.\r\n"), _dma_linked_list_memory_required/1024);
  Serial.printf_P(PSTR("General RAM Available: %d bytes total. Largest free block: %d bytes.\r\n"), 
    heap_caps_get_free_size(MALLOC_CAP_DEFAULT), heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
  #endif

  return true;

} // end allocateDMAmemory()

//DMA линковка дескрипторов буфера
lldesc_t* linkDmaDesc(lldesc_t* dmadesc, lldesc_t* previous_dmadesc, int dmadesk_cnt, uint8_t* dma_buffer, int bufer_size, int max_dma_size)
{
  size_t dma_size;
  while((dmadesk_cnt > 0)&&(bufer_size > 0))
  {
    dma_size = (bufer_size >= max_dma_size)?max_dma_size:bufer_size;
    link_dma_desc(dmadesc, previous_dmadesc, dma_buffer, dma_size);
    previous_dmadesc = dmadesc;
    dmadesc = &dmadesc[1];
    #ifdef SERIAL_DEBUG          
    //Serial.printf_P(PSTR( "BitFrame %d, DMA desk %d, DMA payload of %d bytes. DMA_MAX is %d.\n"), bitframe, current_dmadescriptor_offset, dma_size, DMA_MAX);
    #endif
    dma_buffer += dma_size;
    bufer_size -= dma_size;
    dmadesk_cnt--;
  }
  return previous_dmadesc;
}

//DMA линковка дескрипторов буферов
lldesc_t* linkDmaDesc_n(lldesc_t* dmadesc, lldesc_t* previous_dmadesc, int single_dmadesk_cnt, uint8_t** dma_buffer, int single_bufer_size, int max_dma_size, int buffers_cnt)
{
  int i = 0;
  while (i < buffers_cnt)
  {
    previous_dmadesc = linkDmaDesc(dmadesc, previous_dmadesc, single_dmadesk_cnt, dma_buffer[i], single_bufer_size,max_dma_size);    
    dmadesc = &previous_dmadesc[1];
    i++;
  }
  return previous_dmadesc;
}

//настройка DMA
void MatrixPanel_DMA::configureDMA()
{
  #ifdef SERIAL_DEBUG  
  Serial.println(F("configureDMA(): Starting configuration of DMA engine.\r\n"));
  #endif   

  #ifdef SERIAL_DEBUG  
  Serial.println("configureDMA(): Link dmadesc_prefix.");
  #endif  
  linkDmaDesc_n(dmadesc_prefix, NULL, desc_prefix_cnt, 
    (uint8_t**)&dma_buff.rowBits[offset_prefix], dma_buff.frame_prefix_len*SIZE_DMA_TYPE,DMA_MAX, dma_buff.frame_prefix_cnt);  
   
  #ifdef SERIAL_DEBUG  
  Serial.println("configureDMA(): Link dmadesc_suffix.");
  #endif       
  linkDmaDesc_n(dmadesc_suffix[0], NULL, desc_suffix_cnt, 
    (uint8_t**)&dma_buff.rowBits[offset_suffix], dma_buff.frame_suffix_len*SIZE_DMA_TYPE,DMA_MAX, dma_buff.frame_suffix_cnt);  

  #ifdef SERIAL_DEBUG  
  Serial.println("configureDMA(): Link dmadesc_data.");
  #endif     
  linkDmaDesc_n(dmadesc_data[0], NULL, desc_data_cnt,//<<dma_buff.double_buff, 
    (uint8_t**)dma_buff.rowBits, dma_buff.row_data_len*SIZE_DMA_TYPE,DMA_MAX, dma_buff.all_row_data_cnt);
  
  if (m_cfg.driver == ICN2053)
  {
    #ifdef SERIAL_DEBUG  
    Serial.println("configureDMA(): Link dmadesc_suffix 2.");
    #endif       
    linkDmaDesc_n(dmadesc_suffix[1], NULL, desc_suffix_cnt, 
      (uint8_t**)&dma_buff.rowBits[offset_suffix], dma_buff.frame_suffix_len*SIZE_DMA_TYPE,DMA_MAX, 1);  
    for(int i = ICN2053_DESCEXT_CNT-1; i >= 0; i--)
    {
      //только для инициализации всех полей дескрипторов
      link_dma_desc(&dmadesc_ext[i], NULL, NULL, 0);
    }
    //подготовка регистров конфигурации
    driver_reg = (driver_rgb_t*)heap_caps_calloc(ICN2053_REG_CNT, sizeof(driver_rgb_t), MALLOC_CAP_8BIT);
    if (driver_reg == NULL)
    {
      buffersFree();
      return;
    }
    memcpy(driver_reg, ICN2053_REG_VALUE, ICN2053_REG_CNT*sizeof(driver_rgb_t));
    setDriverRegRGB(&driver_reg[ICN2053_CFG1], (rows_per_frame-1), ICN2053_CFG1_LC_MASK, ICN2053_CFG1_LC_OFFSET);
    icn2053initBuffers(); 
    bufferReady = false;
    bufferBusy = false;
    icn2053_clear = false;
    icn2053_auto_vsync = false;
    getUserRGB = NULL;
    drawUserPixel = NULL;
    driver_cur_reg = 0;
    cur_suffix_id = 0;
    rows_send_cnt = 0;
    dma_int_cnt = 0;
  }else
  {
    //shift
  }
  #ifdef SERIAL_DEBUG  
  Serial.println(F("configureDMA(): Inited buffers.\r\n"));
  #endif       

  //End markers for DMA LL

  #ifdef SERIAL_DEBUG
  Serial.println(F("Performing I2S setup:"));
  #endif

  i2s_parallel_config_t cfg = 
  {
    .gpio_bus = {
      m_cfg.gpio.r1, m_cfg.gpio.g1, m_cfg.gpio.b1, 
      m_cfg.gpio.r2, m_cfg.gpio.g2, m_cfg.gpio.b2, 
      m_cfg.gpio.lat, m_cfg.gpio.oe, 
      m_cfg.gpio.a, m_cfg.gpio.b, m_cfg.gpio.c, m_cfg.gpio.d, m_cfg.gpio.e, 
      -1, -1, -1},
    .gpio_clk = m_cfg.gpio.clk,
    .sample_rate = m_cfg.clk_freq,   
    .sample_width = ESP32_I2S_DMA_MODE,        
    .desccount_a = desc_data_cnt, //использовать только для SHIFT драйверов
    .lldesc_a = dmadesc_data[0],  //      
    .desccount_b = desc_data_cnt, //использовать только для SHIFT драйверов
    .lldesc_b = dmadesc_data[1],  //использовать только для SHIFT драйверов
    .clkphase = m_cfg.clk_phase
  };

  //для Call обработчика DMA  
  MatrixPanel = this;
  // Setup I2S 
  esp_err_t res = i2s_parallel_driver_install(I2S_NUM_1, &cfg);
  if (res != ESP_OK)
  {
    #ifdef SERIAL_DEBUG
    Serial.println("Error setup I2S");  
    #endif
    buffersFree();
    return;
  }
  negative_panel = false;
  scroll_y = 0;
  scroll_x = 0;
  CurColor = 0;
  CurRGB.r = 0;
  CurRGB.g = 0;
  CurRGB.b = 0;

  #ifdef SERIAL_DEBUG
  Serial.println(F("Installed."));
  #endif
  initialized = true;	
  if (m_cfg.driver == ICN2053)
  {
    setShiftCompleteCallback(&icn2053i2sCallback);
    i2s_parallel_send_dma(I2S_NUM_1, dmadesc_suffix[0]);    
    icn2053init();  
  }else
  {
    return;
    //shift
  }

  #ifdef SERIAL_DEBUG  
  Serial.println(F("configureDMA(): DMA setup completed on I2S1.")); 
  #endif         
  initialized = true;	  
  setPanelBrightness(brightness);
} // end initMatrixDMABuff



bool MatrixPanel_DMA::begin()
{
  // Change 'if' to '1' to enable, 0 to not include this Serial output in compiled program        
  #ifdef SERIAL_DEBUG       
  Serial.printf_P(PSTR("Using pin %d for the R1_PIN\n"), m_cfg.gpio.r1);
  Serial.printf_P(PSTR("Using pin %d for the G1_PIN\n"), m_cfg.gpio.g1);
  Serial.printf_P(PSTR("Using pin %d for the B1_PIN\n"), m_cfg.gpio.b1);
  Serial.printf_P(PSTR("Using pin %d for the R2_PIN\n"), m_cfg.gpio.r2);
  Serial.printf_P(PSTR("Using pin %d for the G2_PIN\n"), m_cfg.gpio.g2);
  Serial.printf_P(PSTR("Using pin %d for the B2_PIN\n"), m_cfg.gpio.b2);
  Serial.printf_P(PSTR("Using pin %d for the A_PIN\n"), m_cfg.gpio.a);
  Serial.printf_P(PSTR("Using pin %d for the B_PIN\n"), m_cfg.gpio.b);
  Serial.printf_P(PSTR("Using pin %d for the C_PIN\n"), m_cfg.gpio.c);
  Serial.printf_P(PSTR("Using pin %d for the D_PIN\n"), m_cfg.gpio.d);
  Serial.printf_P(PSTR("Using pin %d for the E_PIN\n"), m_cfg.gpio.e);
  Serial.printf_P(PSTR("Using pin %d for the LAT_PIN\n"), m_cfg.gpio.lat);
  Serial.printf_P(PSTR("Using pin %d for the OE_PIN\n"),  m_cfg.gpio.oe);
  Serial.printf_P(PSTR("Using pin %d for the CLK_PIN\n"), m_cfg.gpio.clk);
  #endif   

  pixels_per_row = m_cfg.mx_width*m_cfg.mx_count_width*m_cfg.mx_count_height;
  rows_per_frame = m_cfg.mx_height/ROWS_IN_PARALLEL;
  driver_cnt = pixels_per_row/DRIVER_BITS;
  brightness = BRIGHTNESS_DEFAULT;

  if (PANEL_COUNT_Y <= 1) virtual_draw = 0;
  if (virtual_draw)
  {
    serpentine_chain = (virtual_draw & _VIRTUAL_BIT_SERPENTINE) != 0;  
    top_down_chain = (virtual_draw & _VIRTUAL_BIT_TOP_DOWN) != 0;
  }else
  {
    serpentine_chain = false;
    top_down_chain = true;
  }
  setRotation(0);
  setMirrorX(false);
  setMirrorY(false);
  #ifdef SERIAL_DEBUG  
  Serial.println(mirror_x);
  Serial.println(mirror_y);
  #endif

  /* As DMA buffers are dynamically allocated, we must allocated in begin()
  * Ref: https://github.com/espressif/arduino-esp32/issues/831
  */
  if ( !allocateDMAmemory() ) 
  {  
    return false; 
  } // couldn't even get the basic ram required.

  // Flush the DMA buffers prior to configuring DMA - Avoid visual artefacts on boot.
  //clearScreen(); // Must fill the DMA buffer with the initial output bit sequence or the panel will display garbage
  // Setup the ESP32 DMA Engine. Sprite_TM built this stuff.
  configureDMA(); //DMA and I2S configuration and setup

  #ifdef SERIAL_DEBUG 
  if (!initialized)    
    Serial.println(F("MatrixPanel_DMA::begin() failed."));
  #endif      

  return initialized;
}

//overload for compatibility
bool MatrixPanel_DMA::begin(hub75_pins_t* hub75_pins_ptr)
{
  // RGB
  memcpy(&m_cfg.gpio, hub75_pins_ptr, sizeof(hub75_pins_t));
  return begin();
}

MatrixPanel_DMA::MatrixPanel_DMA(virtual_matrix_t _virtual_panel):
  #if defined(USE_GFX_ROOT)
  GFX(MATRIX_WIDTH, MATRIX_HEIGHT)
  #elif !defined(NO_GFX)
  Adafruit_GFX((PANEL_COUNT_Y > 1)?(PANEL_WIDTH*PANEL_COUNT_X):(PANEL_WIDTH*PANEL_COUNT_X*PANEL_COUNT_Y), 
  (PANEL_COUNT_Y > 1)?(PANEL_HEIGHT*PANEL_COUNT_Y):(PANEL_HEIGHT))
  #endif
{
  memcpy(&m_cfg, &hub75_cfg_default, sizeof(hub75_cfg_t));
  initialized = false;
  virtual_draw = _virtual_panel;
}

//MatrixPanel_DMA 
//@param  {HUB75_I2S_CFG} opts : structure with matrix configuration
MatrixPanel_DMA::MatrixPanel_DMA(const hub75_cfg_t& opts, virtual_matrix_t _virtual_panel):
  #if defined(USE_GFX_ROOT) 
  GFX(opts.mx_width*opts.chain_length, opts.mx_height),
  #elif !defined(NO_GFX)
  Adafruit_GFX((PANEL_COUNT_Y > 1)?(opts.mx_width*opts.mx_count_width):(opts.mx_width*opts.mx_count_width*opts.mx_count_height), 
               (PANEL_COUNT_Y > 1)?(opts.mx_height*opts.mx_count_height):(opts.mx_height)),
  #endif        
  m_cfg(opts)
{    
  initialized = false;
  virtual_draw = _virtual_panel;
}


//---------------------------------//
//  Функции управления             //
//---------------------------------//

//очистить DMA буфер
void MatrixPanel_DMA::clearDmaBuffer(uint8_t _buff_id)
{
  int y_max, y_min;
  _buff_id &= m_cfg.double_dma_buff;
  if (_buff_id) 
  {
    y_max = dma_buff.all_row_data_cnt - 1; 
    y_min = dma_buff.all_row_data_cnt >> 1;
  }else 
  {
    y_max = (dma_buff.all_row_data_cnt >> 1) - 1; 
    y_min = 0;
  }
  for (int y = y_max; y >= y_min; y--)
  {
    for (int x = dma_buff.row_data_len - 1; x >= 0; x--)
    {
      dma_buff.rowBits[y][x] &= ~BITMASK_RGB12;
    }
  }
}

//очистить фрейм буфер
void MatrixPanel_DMA::clearBuffer(uint8_t _buff_id)
{  
  _buff_id &= 1;
  uint8_t clear_buff_idx = m_cfg.double_buff & (back_buffer_id^_buff_id^1);
  if (frame_buffer.len != 0)
  {    
    if (initialized &(_buff_id == 0))
    {
      panelShowOff();
      //запускаем очистку первого фрейма в памяти драйверов экрана
      waitDmaReady();
      icn2053_clear = true;
      sendFrame();
    }
    if (!m_cfg.double_buff)
      waitDmaReady();
    //очистка первичного видеобуфер параллельно с DMA очисткой драйверов экрана
    if (frame_buffer.frameBits[clear_buff_idx] != NULL)
      memset((uint8_t*)frame_buffer.frameBits[clear_buff_idx], 0, frame_buffer.len * VB_SIZE);
    if (initialized &(_buff_id == 0))
    {      
      //запускаем очистку второго фрейма в памяти драйверов экрана
      waitDmaReady();
      icn2053_clear = true;
      sendFrame();
      panelShowOn();
    }    
  }else
  {
    //shift
    //clearDmaBuffer(clear_buff_idx);
  }
}

//получить конфигурацию
const hub75_cfg_t& MatrixPanel_DMA::getCfg() const 
{
  return m_cfg;
};

//остановить DMA
void MatrixPanel_DMA::stopDMAoutput() 
{  
  clearScreen();
  i2s_parallel_stop_dma(I2S_NUM_1);
} 

//переключить DMA буфер
IRAM_ATTR void MatrixPanel_DMA::flipBuffer() 
{         
  if (!m_cfg.double_buff)
  {
    if (frame_buffer.len != 0) sendFrame();
  }else
  {
    #ifdef SERIAL_DEBUG     
    //Serial.printf_P(PSTR("Set show buffer to: %d\n"), back_buffer_id);
    #endif      
    if (frame_buffer.len != 0)
    {
      //waitDmaReady();
      waitDmaReady();
      back_buffer_id ^= 1;
      sendFrame();
    }else
    {
      //shift
    }
  }      
}

//переключить DMA буфер
IRAM_ATTR void MatrixPanel_DMA::flipDMABuffer() 
{
  flipBuffer();
}


void MatrixPanel_DMA::setDMAGetUserRGB(getUserRGB_p _getUserRGB)
{
  getUserRGB = _getUserRGB;
}

void MatrixPanel_DMA::setDrawUserPixel(drawUserPixel_p _drawUserPixel)
{
  drawUserPixel = _drawUserPixel;
}




